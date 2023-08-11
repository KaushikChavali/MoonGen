-- LTE emulation script that supports Handovers (HOs) and variable bitrate and delay emulation over time.
-- The features are primarily designed for aerial use-cases.
-- Script extended from the source https://github.com/brentondwalker/MoonGen/blob/master/examples/lte-emulator.lua.

-- cellular-satcom-emulator : Multipath Cellular and Satellite Emulation Testbed
-- Copyright (C) 2023 Kaushik Chavali
-- 
-- This file is part of the cellular-satcom-emulator.
-- https://github.com/KaushikChavali/cellular-satcom-emulator
--
-- cellular-satcom-emulator is free software: you can redistribute it 
-- and/or modify it under the terms of the GNU General Public License 
-- as published by the Free Software Foundation, either version 3 of 
-- the License, or (at your option) any later version.
-- 
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
-- 
-- You should have received a copy of the GNU General Public License
-- along with this program.  If not, see <https://www.gnu.org/licenses/>.

local mg      = require "moongen"
local memory  = require "memory"
local device  = require "device"
local ts      = require "timestamping"
local stats   = require "stats"
local log     = require "log"
local limiter = require "software-ratecontrol"
local pipe    = require "pipe"
local ffi     = require "ffi"
local libmoon = require "libmoon"
local histogram = require "histogram"

local namespaces = require "namespaces"

local PKT_SIZE	= 60

local LOG_LEVEL = "DEBUG"

-- 
-- Network A --> MG --> C
--
-- Uplink example:
-- sudo ./build/MoonGen examples/l2-forward-psring-hybrid-latency-rate-lte-catchup-rateramp.lua -d <A> <C> -r 38 40 -l 10 30 -q 1000 350 -u 1000 1000 -c 0.005 0.005 -o 0.001 0.001
--
-- Downlink example:
-- sudo ./build/MoonGen examples/l2-forward-psring-hybrid-latency-rate-lte-catchup-rateramp.lua -d <C> <A> -r 38 40 -l 10 30 -q 1000 350 -u 1000 1000 -c 0.005 0.005 -o 0.001 0.001
--

function configure(parser)
	parser:description("Forward traffic between interfaces with moongen rate control")
	parser:description("device args:    dev1=eNodeB     dev2=UE Handset")
	parser:option("-d --dev", "Devices to use, specify the same device twice to echo packets."):args(2):convert(tonumber):default({0,1})
	parser:option("-r --rate", "Forwarding rates in Mbps (two values for two links)"):args(2):convert(tonumber):default({40,30})
	parser:option("-t --threads", "Number of threads per forwarding direction using RSS."):args(1):convert(tonumber):default(1)
	parser:option("-l --latency", "Fixed emulated latency (in ms) on the link."):args(2):convert(tonumber):default({40,40})
	parser:option("-x --xlatency", "Extra exponentially distributed latency, in addition to the fixed latency (in ms)."):args(2):convert(tonumber):default({0,0})
	parser:option("-q --queuedepth", "Maximum number of packets to hold in the delay line"):args(2):convert(tonumber):default({0,0})
	parser:option("-o --loss", "Rate of packet drops"):args(2):convert(tonumber):default({0,0})
	parser:option("-c --concealedloss", "Rate of concealed packet drops"):args(2):convert(tonumber):default({0,0})
	parser:option("-u --catchuprate", "After a concealed loss, this rate will apply to the backed-up frames."):args(2):convert(tonumber):default({0,0})
	parser:option("--short_DRX_cycle_length", "The short DRX cycle length in ms"):args(1):convert(tonumber):default(6)
	parser:option("--long_DRX_cycle_length", "The long DRX cycle length in ms"):args(1):convert(tonumber):default(12)
	parser:option("--active_time", "The active time from PDCCH in ms"):args(1):convert(tonumber):default(1)
	parser:option("--continuous_reception_inactivity_timer", "The continous reception inactivity timer in ms"):args(1):convert(tonumber):default(200)
	parser:option("--short_DRX_inactivity_timer", "The short DRX inactivity timer in ms"):args(1):convert(tonumber):default(2298)
	parser:option("--long_DRX_inactivity_timer", "The long DRX inactivity timer in ms"):args(1):convert(tonumber):default(7848)
	parser:option("--rcc_idle_cycle_length", "The RCC IDLE cycle length in ms"):args(1):convert(tonumber):default(50)
	parser:option("--rcc_connection_build_delay", "The Delay from RCC_IDLE to RCC_CONNECT in ms"):args(1):convert(tonumber):default(70)
	parser:option("--variable_delay", "The variable latencies over time file path for UL,DL"):args(2):convert(tostring):default({"",""})
	parser:option("--variable_rate", "The variable forwarding rates over time file path for UL,DL"):args(2):convert(tostring):default({"",""})
	
	parser:option("--ho_pcm", "The number of people/cell/minute, used for calculating the handover interruption time (HIT)"):args(1):convert(tonumber):default(2.5)
	parser:option("--ho_frequency", "Handovers per second: mean and variance"):args(2):convert(tonumber):default({0,0})
	return parser:parse()
end


function master(args)
	
	log:setLevel(LOG_LEVEL)

	-- configure devices
	for i, dev in ipairs(args.dev) do
		args.dev[i] = device.config{
			port = dev,
			txQueues = args.threads,
			rxQueues = args.threads,
			rssQueues = 0,
			rssFunctions = {},
			txDescs = 128,
			dropEnable = true,
			disableOffloads = true
		}
	end
	device.waitForLinks()

	-- print stats
	stats.startStatsTask{devices = args.dev}
	
	-- create the ring buffers
	-- should set the size here, based on the line speed and latency, and maybe desired queue depth
	local qdepthDL = args.queuedepth[1]
	if qdepthDL < 1 then
		qdepthDL = math.floor((args.latency[1] * args.rate[1] * 1000)/672)
	end
	local qdepthUL = args.queuedepth[2]
	if qdepthUL < 1 then
		qdepthUL = math.floor((args.latency[2] * args.rate[2] * 1000)/672)
	end

	log:debug("Queue depths: ringDL=%d, ringUL=%d", qdepthDL, qdepthUL)
	if qdepthDL == 0 or qdepthUL == 0 then
		log:fatal("Calculated queue depth is 0: ringDL=%d, ringUL=%d", qdepthDL, qdepthUL)
	else
		log:info("Use qdepthDL=%d and qdepthUL=%d", qdepthDL, qdepthUL)

	local ringDL = pipe:newPktsizedRing(qdepthDL) -- stores packets that are sent from the eNB to the UE
	local ringUL = pipe:newPktsizedRing(qdepthUL) -- stores packets that are sent from the UE to the eNB

	local lte_ns = namespaces:get("lte")

	-- parse the variable delay parameters from the file for UL and DL directions
	local latency_ul = parseVariableDelayBitrateFile(args.variable_delay[1])
	local latency_dl = parseVariableDelayBitrateFile(args.variable_delay[2])

	-- parse the variable bitrate parameters from the file for UL/DL directions
	local bitrate_ul = parseVariableDelayBitrateFile(args.variable_rate[1])
	local bitrate_dl = parseVariableDelayBitrateFile(args.variable_rate[2])

	-- start the forwarding tasks
	for i = 1, args.threads do
		-- DOWNLINK (eNB -> UE)
		mg.startTask("forward", 1, ringDL, args.dev[1]:getTxQueue(i - 1), args.dev[1],
				args.rate[1], 
			 	args.latency[1],
				bitrate_dl,
				latency_dl,
				args.xlatency[1],
				args.loss[1],
				args.concealedloss[1],
				args.catchuprate[1],
				args.short_DRX_cycle_length,
				args.long_DRX_cycle_length,
				args.active_time,
				args.continuous_reception_inactivity_timer,
				args.short_DRX_inactivity_timer,
				args.long_DRX_inactivity_timer,
				args.rcc_idle_cycle_length,
				args.rcc_connection_build_delay,
				args.ho_pcm,
				args.ho_frequency)
		-- UPLINK (UE -> eNB) does not have to wait for a RCC_IDLE cycle, so the rcc_idle_cycle_length is effectively zero
		if args.dev[1] ~= args.dev[2] then
			mg.startTask("forward", 2, ringUL, args.dev[2]:getTxQueue(i - 1), args.dev[2],
					args.rate[2],
					args.latency[2],
					bitrate_ul,
					latency_ul,
					args.xlatency[2],
					args.loss[2],
					args.concealedloss[2],
					args.catchuprate[2],
					args.short_DRX_cycle_length,
					args.long_DRX_cycle_length,
					args.active_time,
					args.continuous_reception_inactivity_timer,
					args.short_DRX_inactivity_timer,
					args.long_DRX_inactivity_timer,
					0,
					args.rcc_connection_build_delay,
					args.ho_pcm,
					args.ho_frequency)
		end
	end

	-- start the receiving/latency tasks
	for i = 1, args.threads do
		mg.startTask("receive", 1, ringDL, args.dev[2]:getRxQueue(i - 1), args.dev[2])
		if args.dev[1] ~= args.dev[2] then
			mg.startTask("receive", 2, ringUL, args.dev[1]:getRxQueue(i - 1), args.dev[1])
		end
	end

	mg.waitForTasks()
end

function receive(threadNumber, ring, rxQueue, rxDev)

	log:setLevel(LOG_LEVEL)
	log:debug("Receive thread : %d", threadNumber)

	local tsc_hz = libmoon:getCyclesFrequency()
	local tsc_hz_ms = tsc_hz / 1000
	local tsc_hz_ns = tsc_hz / 1000000

	local bufs = memory.createBufArray()
	local count = 0
	--local count_hist = histogram:new()
	while mg.running() do
		count = rxQueue:recv(bufs)

		--count_hist:update(count)
		for iix=1,count do
			local buf = bufs[iix]
			local ts = limiter:get_tsc_cycles()
			buf.udata64 = ts
		end

		if count > 0 then
			pipe:sendToPktsizedRing(ring.ring, bufs, count)
		end
	end
	--count_hist:print()
	--count_hist:save("rxq-pkt-count-distribution-histogram-"..rxDev["id"]..".csv")
end

function sample_gaussian(mean, variance)
	-- Taken from https://rosettacode.org/wiki/Statistics/Normal_distribution#Lua
    return math.sqrt(-2 * variance * math.log(math.random())) *
           math.cos(2 * math.pi * math.random()) + mean
end

-- Help function:
-- parse variable delay and bitrate values over time from config file
function parseVariableDelayBitrateFile(path)

	local delay_bitrate_table = {}

	if path == nil or path == '' then
		return delay_bitrate_table
	end

	for line in io.lines(path) do
		 local tsc_in_sec, delay_bitrate_val = line:match("%s*(.*),%s*(.*)")
                 delay_bitrate_table[tonumber(tsc_in_sec)] = tonumber(delay_bitrate_val)
        end

	return delay_bitrate_table

end

function forward(threadNumber, ring, txQueue, txDev,
			rate,
			latency,
			bitrate_arr,
			latency_arr,
			xlatency,
			lossrate,
			clossrate,
			catchuprate,
			short_DRX_cycle_length,
			long_DRX_cycle_length,
			active_time,
			continuous_reception_inactivity_timer,
			short_DRX_inactivity_timer,
			long_DRX_inactivity_timer,
			rcc_idle_cycle_length,
			rcc_connection_build_delay,
			ho_pcm,
			ho_frequency)

	log debug = LOG_LEVEL = "DEBUG"
	log:setLevel(LOG_LEVEL)

	local numThreads = 1

	local lte_ns = namespaces:get("lte")

	math.randomseed( os.time() )

	local linkspeed = txDev:getLinkStatus().speed

	local tsc_hz = libmoon:getCyclesFrequency()
	local tsc_hz_ms = tsc_hz / 1000

	local debug = true

	log:info("Thread %d: forward with rate %dMbps, latency %dms, loss rate %f, clossrate %f and catchuprate %f. linkspeed=%d, tsc_hz=%d",
			 threadNumber, rate, latency, lossrate, clossrate, catchuprate, linkspeed, tsc_hz)

	-- DRX in LTE is in RRC_IDLE or in RRC_CONNECTED mode
	-- RRC_IDLE: sleep state
	-- RRC_CONNECTED:
	lte_ns.rcc_idle = true

	-- after connection establishment the rate ramps up to the emulated rate over some time
	lte_ns.rate_ramp_mode = false
	local rate_ramp_mode_last = false  -- for the other thread(s) to detect when we enter rate_ramp_mode
	local rate_ramp_length_ms = 500
	local rate_ramp_length_tsc_hz = rate_ramp_length_ms * tsc_hz_ms
	local rate_ramp_start = 0
	local rate_ramp_end = 0
	local rate_ramp_min = 5
	if rate_ramp_min > rate then
		log:fatal("rate_ramp_min=5 is greater than rate=%d", rate)
	end

	-- the RRC_CONNECTED mode got the short DRX cycle and long DRX cycle
	lte_ns.short_DRX = false
	lte_ns.long_DRX = false
	lte_ns.continuous_reception = false

	local last_activity = limiter:get_tsc_cycles()
	lte_ns.last_packet_time = ullToNumber(last_activity)
	lte_ns.last_packet_time_thread1 = lte_ns.last_packet_time
	lte_ns.last_packet_time_thread2 = lte_ns.last_packet_time

	lte_ns.first_rcc_connected = false

	lte_ns.ho_start = ullToNumber(limiter:get_tsc_cycles())
	lte_ns.ho_end = ullToNumber(limiter:get_tsc_cycles())

	local bufs = memory.createBufArray()
	local count = 0

	-- when there is a concealed loss, the backed-up packets can catch up at line rate
	local catchup_mode = false

	-- between 0.32 and 2.56 sec
	local rcc_idle_cycle_length_tsc_hz_ms = rcc_idle_cycle_length * tsc_hz_ms

	local short_DRX_cycle_length_tsc_hz_ms = short_DRX_cycle_length * tsc_hz_ms
	local long_DRX_cycle_length_tsc_hz_ms = long_DRX_cycle_length * tsc_hz_ms

	local active_time_tsc_hz_ms = active_time * tsc_hz_ms

	-- will be reset after each send/received packet
	-- timer is between 1ms - 2.56sec Paper-[10]
	local inactive_continuous_reception_cycle_time = continuous_reception_inactivity_timer * tsc_hz_ms

	local inactive_short_DRX_cycle_time = (short_DRX_inactivity_timer + continuous_reception_inactivity_timer) * tsc_hz_ms

	local inactive_long_DRX_cycle_time = (long_DRX_inactivity_timer + short_DRX_inactivity_timer + continuous_reception_inactivity_timer)* tsc_hz_ms

	-- 16 to 19 signalling messages
	local rcc_connection_build_delay_tsc_hz_ms = rcc_connection_build_delay * tsc_hz_ms

	-- in ms
	local concealed_resend_time = 8
	local num_catchup_packets = 0

	-- send time of the most recent concealed loss
	local cl_send_time = 0

	-- when catching up after a concealed loss, we need to keep track of when a packet
	-- /would/ have been sent during the correction delay interval
	local last_send_time = 0
	local last_tx_time_tsc = 0

	-- time between when a concealed loss would have been sent, and when it is actually sent
	local clI = 0

	local time_stuck_in_loop = 0

	-- handover parameters modeled as a Gaussian Mixture Model (GMM) using Estimation Maximization (EM) algorithm
	local mu = {9.3879, 1.9681, 3.9050, 203.1832}
	local sigma = {5.6246, 0.2484, 0.9414, 21.2318}
	local p_i = {0.0095, 0.8383, 0.1214, 0.0174}

	-- log forward thread's start time to track emulation progress
	local fwd_start_time = ullToNumber(limiter:get_tsc_cycles())
	-- set initial index of latency/bitrate arrays
	local lat_idx = 0

	-- set initial latency and bitrate
	if next(latency_arr) ~= nil then
		local latency = latency_arr[lat_idx]
	end
	if next(bitrate_arr) ~= nil then
		local rate = bitrate_arr[lat_idx]
	end

	while mg.running() do
		-- RCC_IDLE to RCC_CONNECTED the delay
		if lte_ns.first_rcc_connected then
			if debug then print("Build RCC_CONNECTION "..threadNumber) end
			last_activity = limiter:get_tsc_cycles()
			while limiter:get_tsc_cycles() < last_activity + rcc_connection_build_delay_tsc_hz_ms do
				if not mg.running() then
					return
				end
				-- if the other thread finished the LOOP
				if not lte_ns.first_rcc_connected then
					break
				end
			end
			lte_ns.rate_ramp_mode = true
			rate_ramp_mode_last = true
			rate_ramp_start = limiter:get_tsc_cycles()
			rate_ramp_end = rate_ramp_start + rate_ramp_length_tsc_hz
			lte_ns.first_rcc_connected = false
			if time_stuck_in_loop > 0 then
				time_stuck_in_loop = time_stuck_in_loop + rcc_connection_build_delay_tsc_hz_ms
			end
			last_activity = limiter:get_tsc_cycles()
		end

		-- detect if another thread put us into rate_ramp_mode
		if (lte_ns.rate_ramp_mode and not rate_ramp_mode_last) then
			rate_ramp_start = limiter:get_tsc_cycles()
			rate_ramp_end = rate_ramp_start + rate_ramp_length_tsc_hz
		end
		rate_ramp_mode_last = lte_ns.rate_ramp_mode

		-- modify latency/bitrate array index and associated values w.r.t to forward thread's initial ts
		local diff_tsc = ullToNumber(limiter:get_tsc_cycles()) - fwd_start_time
		local curr_tsc = lat_idx * tsc_hz
		-- log:debug("%d, Diff ts %d, Curr ts %d", threadNumber, diff_tsc, curr_tsc)
		if diff_tsc > curr_tsc and not lte_ns.rate_ramp_mode then
				-- print (os.date ("%c"))
				-- log:debug("%d, idx %d, latency %d, bandwidth %d", threadNumber, lat_idx, latency, rate)
				lat_idx = lat_idx + 1
		end
		
		-- update latency values for variable delay over time
		if next(latency_arr) ~= nil then
			latency = latency_arr[lat_idx]
		end
		
		-- update bitrate values for variable rate over time
		if next(bitrate_arr) ~= nil then
			rate = bitrate_arr[lat_idx]
		end

		-- if the continuous_reception mode is active
		if lte_ns.continuous_reception then
			count = pipe:recvFromPktsizedRing(ring.ring, bufs, 1)

			if (ho_frequency[1] > 0 and lte_ns.ho_end + tsc_hz < limiter:get_tsc_cycles()) then
				-- Frequency in handover per second: tuple of mean and variance
				local sec_until_next_ho = sample_gaussian(ho_frequency[1], ho_frequency[2])
				lte_ns.ho_start = ullToNumber(limiter:get_tsc_cycles()) + sec_until_next_ho * tsc_hz
				-- Parameters taken from han2015hoMeas (in ms)
				-- local hit_duration = sample_gaussian(0.39 * ho_pcm + 16.34, (0.21 * ho_pcm + 0.27)^2)

				-- HIT duration computed from Gaussian Mixture Model (LTE Air Measurements)
				math.randomseed(os.time())
				local uniform_random = math.random()
				log:info("Random variable generated from uniform distribution : %f", uniform_random)

				local hit_duration = 0
				local idx = 0
				if ( uniform_random < p_i[1] ) then
					idx = 1
					hit_duration = sample_gaussian(mu[1], sigma[1] ^ 2)
				elseif ( uniform_random < (p_i[1] + p_i[2]) ) then
					idx = 2
					hit_duration = sample_gaussian(mu[2], sigma[2] ^ 2)
				elseif ( uniform_random < (p_i[1] + p_i[2] + p_i[3]) ) then
					idx = 3
					hit_duration = sample_gaussian(mu[3], sigma[3] ^ 2)
				else
					idx = 4
					hit_duration = sample_gaussian(mu[4], sigma[4] ^ 2)
				end

				log:info("Sampled from a Gaussian with Mean : %f and Variance : %f", mu[idx], sigma[idx])
				
				lte_ns.ho_end = lte_ns.ho_start + hit_duration * tsc_hz_ms
				log:debug("[%d] Next handover in %.2f seconds for %.6f ms", threadNumber, sec_until_next_ho, hit_duration)
			end

			for iix=1,count do
				local buf = bufs[iix]

				-- get the buf's arrival timestamp and compare to current time
				--local arrival_timestamp = buf:getTimestamp()
				local arrival_timestamp = buf.udata64
				local extraDelay = 0.0
				if (xlatency > 0) then
					extraDelay = -math.log(math.random())*xlatency
				end

				-- emulate concealed losses
				-- for now, only allow one concealed loss at a time
				local closses = 0.0
				if (not catchup_mode) then
				while (math.random() < clossrate) do
					closses = closses + 1
					if (catchuprate > 0) then
						catchup_mode = true
					end
				end
				end

				local remaining_hit = 0
				if lte_ns.ho_start < limiter:get_tsc_cycles() and lte_ns.ho_end > limiter:get_tsc_cycles() then
					remaining_hit = lte_ns.ho_end - limiter:get_tsc_cycles()
					log:debug("[%d, %d] Handover for %d ticks", threadNumber, ullToNumber(limiter:get_tsc_cycles()), ullToNumber(remaining_hit))
				end

				local wait_time = (closses*concealed_resend_time + latency + extraDelay) * tsc_hz_ms
				local send_time = arrival_timestamp + wait_time + remaining_hit + time_stuck_in_loop
				if remaining_hit > 0 then
					log:debug("[%d] %d < %d + %d + %d + %d", threadNumber,
							  ullToNumber(limiter:get_tsc_cycles()),
							  ullToNumber(arrival_timestamp), wait_time, ullToNumber(remaining_hit), time_stuck_in_loop)
				end

				-- spin/wait until it is time to send this frame
				-- this assumes frame order is preserved
				local send_time_cycles = 0
				while limiter:get_tsc_cycles() < send_time do
					send_time_cycles = send_time_cycles + 1
					if not mg.running() then
						return
					end
				end
				-- if send_time_cycles > 0 then
				-- 	log:debug("[%d] did not wait (%d) %d < %d + %d + ? + ?", threadNumber,
				-- 			  ullToNumber(limiter:get_tsc_cycles() - (arrival_timestamp + wait_time)),
				-- 			  ullToNumber(limiter:get_tsc_cycles()),
				-- 			  ullToNumber(arrival_timestamp), wait_time)
				-- end

				local current_rate = rate
				if lte_ns.rate_ramp_mode then
					local tsc_now = limiter:get_tsc_cycles()
					if (tsc_now > rate_ramp_end) then
						lte_ns.rate_ramp_mode = false
						rate_ramp_start = 0
						rate_ramp_end = 0
					else
						local rate_range = rate - rate_ramp_min
						local ramp_fraction = (tsc_now - rate_ramp_start) / rate_ramp_length_tsc_hz
						current_rate = rate_ramp_min + rate_range * (tsc_now - rate_ramp_start) / rate_ramp_length_tsc_hz
						log:info("In rate ramp mode: set current_rate=%s", current_rate)
					end
				end

				if(closses > 0) then
					-- compute how much time other packets have left to transmit while
					-- the concealed loss is getting detected/corrected
					cl_send_time = send_time
					local pktSize = buf.pkt_len + 24
					local time_to_tx_ms = ((pktSize*8) / (current_rate*1000))
					clI = (closses*concealed_resend_time - time_to_tx_ms) * tsc_hz_ms
				end

				-- if we're in catchup_mode, check if the current packet fits inside the clI window or not
				-- for now, just check if its normal send_time comes before the delayed send time of the concelaed lost packet
				if (closses == 0 and catchup_mode) then
					local pktSize = buf.pkt_len + 24
					local time_to_tx_tsc = ((pktSize*8) / (current_rate*1000)) * tsc_hz_ms

					if (send_time < (last_send_time + last_tx_time_tsc)) then
						send_time = last_send_time + last_tx_time_tsc
					end

					if (send_time > cl_send_time) then
						-- we exit catchup_mode
						num_catchup_packets = 0
						catchup_mode = false
						last_send_time = 0
						last_tx_time_tsc = 0
					elseif (time_to_tx_tsc > (cl_send_time-send_time)) then
						num_catchup_packets = 0
						catchup_mode = false
						last_send_time = 0
						last_tx_time_tsc = 0
					else
						clI = cl_send_time - send_time - time_to_tx_tsc
						last_send_time = send_time
						last_tx_time_tsc = time_to_tx_tsc
					end
				end

				time_stuck_in_loop = 0

				local pktSize = buf.pkt_len + 24
				if (catchup_mode) then
					buf:setDelay((pktSize) * (linkspeed/catchuprate - 1))
					num_catchup_packets = num_catchup_packets + 1
				else
					buf:setDelay((pktSize) * (linkspeed/current_rate - 1))
				end
			end

			if count > 0 then

				-- the rate here doesn't affect the result afaict.  It's just to help decide the size of the bad pkts
				txQueue:sendWithDelayLoss(bufs, rate * numThreads, lossrate, count)

				last_activity = limiter:get_tsc_cycles()
				--lte_ns.last_packet_time = ullToNumber(limiter:get_tsc_cycles())
				local tmptime = ullToNumber(limiter:get_tsc_cycles())
				if (threadnumber == 1) then
					lte_ns.last_packet_time_thread1 = tmptime
				else
					lte_ns.last_packet_time_thread2 = tmptime
				end
				--lte_ns.last_packet_time = tmptime
			end

			if limiter:get_tsc_cycles() > last_activity + inactive_continuous_reception_cycle_time then
				local loc_last_packet_time = math.max(lte_ns.last_packet_time_thread1,lte_ns.last_packet_time_thread2)
				if limiter:get_tsc_cycles() > loc_last_packet_time + inactive_continuous_reception_cycle_time then

					if debug then print("continuous_reception deactivating "..threadNumber) end
					lte_ns.continuous_reception = false

					if debug then  print("short_DRX activating "..threadNumber) end
					lte_ns.short_DRX = true
				end
			end
		end

		-- RCC_CONNECTED short_DRX
		if lte_ns.short_DRX then

			last_activity = limiter:get_tsc_cycles()

			local packet_arrival_time = 0
			local lcount = 0
			time_stuck_in_loop = 0

			-- time to wait
			while lte_ns.short_DRX and limiter:get_tsc_cycles() < last_activity + short_DRX_cycle_length_tsc_hz_ms - active_time_tsc_hz_ms do
				lcount = pipe:countPktsizedRing(ring.ring)
				if (lcount > 0) and (packet_arrival_time == 0) then
					packet_arrival_time = limiter:get_tsc_cycles()
				end
				if not mg.running() then
					return
				end
			end

			-- save the time the packet waited
			last_activity = limiter:get_tsc_cycles()
			if (lcount > 0) then
				time_stuck_in_loop = (last_activity-packet_arrival_time)
			end

			-- T_on is active
			while lte_ns.short_DRX and limiter:get_tsc_cycles() < last_activity + active_time_tsc_hz_ms do
				if not mg.running() then
					return
				end
				-- count = pipe:recvFromPktsizedRing(ring.ring, bufs, 1)
				count = pipe:countPktsizedRing(ring.ring)

				if count > 0 then
					if debug then  print("short_DRX deactivating "..threadNumber) end
					lte_ns.short_DRX = false

					if debug then  print("continuous_reception activating "..threadNumber) end
					lte_ns.continuous_reception = true

					last_activity = limiter:get_tsc_cycles()
					--lte_ns.last_packet_time = ullToNumber(limiter:get_tsc_cycles())
					local tmptime = ullToNumber(limiter:get_tsc_cycles())
					if (threadnumber == 1) then
						lte_ns.last_packet_time_thread1 = tmptime
					else
						lte_ns.last_packet_time_thread2 = tmptime
					end
					--lte_ns.last_packet_time = tmptime
					break
				end
			end

			-- if the the max of interactive Time from short DRX arrived, it will be changed to long DRX
			local loc_last_packet_time = math.max(lte_ns.last_packet_time_thread1,lte_ns.last_packet_time_thread2)
			if limiter:get_tsc_cycles() > loc_last_packet_time + inactive_short_DRX_cycle_time then
				if debug then  print("short_DRX deactivating after inactive time, "..threadNumber) end
				lte_ns.short_DRX = false

				if debug then  print("long_DRX activating after inactive time, "..threadNumber) end
				lte_ns.long_DRX = true
			end
		end

		-- RCC_CONNECTED long_DRX
		if lte_ns.long_DRX then
			last_activity = limiter:get_tsc_cycles()

			local packet_arrival_time = 0
			local lcount = 0
			time_stuck_in_loop = 0

			-- time to wait
			while lte_ns.long_DRX and limiter:get_tsc_cycles() < last_activity + long_DRX_cycle_length_tsc_hz_ms - active_time_tsc_hz_ms do
				lcount = pipe:countPktsizedRing(ring.ring)
				if (lcount > 0) and (packet_arrival_time == 0) then
					packet_arrival_time = limiter:get_tsc_cycles()
				end
				if not mg.running() then
					return
				end
			end

			-- save the time the packet waited
			last_activity = limiter:get_tsc_cycles()
			if (lcount > 0) then
				time_stuck_in_loop = (last_activity-packet_arrival_time)
			end

			-- T_on is active
			while lte_ns.long_DRX and limiter:get_tsc_cycles() < last_activity + active_time_tsc_hz_ms do
				if not mg.running() then
					return
				end

				count = pipe:countPktsizedRing(ring.ring)

				if count > 0 then
					if debug then  print("long_DRX deactivating "..threadNumber) end
					lte_ns.long_DRX = false

					if debug then  print("continuous_reception activating "..threadNumber) end
					lte_ns.continuous_reception = true

					last_activity = limiter:get_tsc_cycles()
					--lte_ns.last_packet_time = ullToNumber(limiter:get_tsc_cycles())
					local tmptime = ullToNumber(limiter:get_tsc_cycles())
					if (threadnumber == 1) then
						lte_ns.last_packet_time_thread1 = tmptime
					else
						lte_ns.last_packet_time_thread2 = tmptime
					end

					--lte_ns.last_packet_time = tmptime

					break
				end
			end

			-- if the the max of interactive Time from long DRX arrived, return to RCC_IDLE
			local loc_last_packet_time = math.max(lte_ns.last_packet_time_thread1,lte_ns.last_packet_time_thread2)
			if limiter:get_tsc_cycles() > loc_last_packet_time + inactive_long_DRX_cycle_time then

				if debug then print("long_DRX deactivating after inactive time, "..threadNumber) end
				lte_ns.long_DRX = false

				if debug then  print("rcc_idle activating after inactive time, "..threadNumber) end
				lte_ns.rcc_idle = true
			end
		end

		-- if the RCC_IDLE mode is active
		if lte_ns.rcc_idle then
			last_activity = limiter:get_tsc_cycles()

			local packet_arrival_time = 0
			local lcount = 0
			time_stuck_in_loop = 0

			-- time to wait
			if (rcc_idle_cycle_length_tsc_hz_ms > 0) then
				while limiter:get_tsc_cycles() < last_activity + rcc_idle_cycle_length_tsc_hz_ms - active_time_tsc_hz_ms do
					lcount = pipe:countPktsizedRing(ring.ring)
					if (lcount > 0) and (packet_arrival_time == 0) then
						packet_arrival_time = limiter:get_tsc_cycles()
					end
					if not mg.running() then
						return
					end
				end
			end

			-- save the time the packet waited
			last_activity = limiter:get_tsc_cycles()
			if (lcount > 0) then
				time_stuck_in_loop = (last_activity - packet_arrival_time)
			end

			-- T_on is active
			while limiter:get_tsc_cycles() < last_activity + active_time_tsc_hz_ms do
				if not mg.running() then
					return
				end

				-- this does not actually count the contents,
				-- it computes the difference between head and tail
				count = pipe:countPktsizedRing(ring.ring)

				if count > 0 then

					if debug then print("rcc_idle deactivating "..threadNumber) end
					lte_ns.rcc_idle = false

					if debug then print("continuous_reception activating "..threadNumber) end
					lte_ns.continuous_reception = true

					lte_ns.first_rcc_connected = true

					local tmptime = ullToNumber(limiter:get_tsc_cycles())
					if (threadnumber == 1) then
						lte_ns.last_packet_time_thread1 = tmptime
					else
						lte_ns.last_packet_time_thread2 = tmptime
					end
					--lte_ns.last_packet_time = tmptime

					break
				end
			end
		end
	end
end

-- Help function:
-- cast a uint64_i to "lua number"
function ullToNumber(value)

	local vstring = tostring(value)
	-- remove the "ULL" ending
	vstring = string.sub(vstring, 0, string.len(vstring) - 3)

	return tonumber(vstring)
end
