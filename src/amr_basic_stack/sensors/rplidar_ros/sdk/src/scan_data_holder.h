/*
 * Slamtec LIDAR SDK
 *
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
 /*
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
  * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
  * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  */

#pragma once

#include <algorithm>
#include <atomic>
#include <vector>
#include "sdkcommon.h"
#include "hal/event.h"
#include "hal/locker.h"
#include "sl_lidar_driver.h"

namespace sl {
    class ScanDataHolder
    {
    public:
        ScanDataHolder(size_t maxcount = 8192)
            : _scan_node_buffer_size(maxcount)
            , _scan_node_available_id(-1)
            , _new_scan_ready(false)
        {
            _scanbuffer[0].reserve(_scan_node_buffer_size);
            _scanbuffer[1].reserve(_scan_node_buffer_size);

            memset(_scan_begin_timestamp_uS, 0, sizeof(_scan_begin_timestamp_uS));
        }

        size_t getMaxCacheCount() const {
            return _scan_node_buffer_size;
        }

        void reset() {
            rp::hal::AutoLocker l(_locker);
            _scan_node_available_id = -1;
            _new_scan_ready = false;
            _scanbuffer[0].clear();
            _scanbuffer[1].clear();
            _data_waiter.set(false);
            memset(_scan_begin_timestamp_uS, 0, sizeof(_scan_begin_timestamp_uS));
        }

        bool checkNewScanSignalAndReset()
        {
            return _new_scan_ready.exchange(false);
        }

        void pushScanNodeData(_u64 currentSampleTsUs, const sl_lidar_response_measurement_node_hq_t* hqNode)
        {
            rp::hal::AutoLocker l(_locker);

            int  operationBufID = _getOperationBufferID_locked();
            auto operationalBuf = &_scanbuffer[operationBufID];

            if (hqNode->flag & RPLIDAR_RESP_HQ_FLAG_SYNCBIT) {
                if (operationalBuf->size()) {
                    operationBufID = _finishCurrentScanAndSwap_locked();
                    operationalBuf = &_scanbuffer[operationBufID];

                    // publish the available scan
                    _new_scan_ready = true;
                    _data_waiter.set();

                }

                assert(operationalBuf->size() == 0);

                //store the timestamp info
                _scan_begin_timestamp_uS[operationBufID] = currentSampleTsUs;
            }
            else {
                if (operationalBuf->size() == 0) {
                    //discard the data, do not form partial scan
                    return;
                }
            }

            if (operationalBuf->size() >= _scan_node_buffer_size) {
                //replace the last entry if buffer is full
                operationalBuf->at(operationalBuf->size() - 1) = {*hqNode, currentSampleTsUs};
            }
            else {
                operationalBuf->push_back({*hqNode, currentSampleTsUs});
            }

        }

        void rewindCurrentScanData() {
            rp::hal::AutoLocker l(_locker);
            _getOperationalBuffer_locked().clear();
        }

        sl_result readScan(sl_lidar_response_measurement_node_hq_t* nodes, TimedHqNode* timed,
                           size_t& count, sl_u64* scanStart, sl_u32 timeout)
        {
            if (!nodes && !timed) return SL_RESULT_INVALID_DATA;

            if (_data_waiter.wait(timeout) != rp::hal::Event::EVENT_OK)
                return SL_RESULT_OPERATION_TIMEOUT;

            rp::hal::AutoLocker lock(_locker);
            if (_scan_node_available_id < 0) return SL_RESULT_OPERATION_TIMEOUT;
            const auto & scan = _scanbuffer[_scan_node_available_id];
            _new_scan_ready = false;
            if (scanStart) *scanStart = _scan_begin_timestamp_uS[_scan_node_available_id];
            count = std::min(count, scan.size());
            if (timed) {
                std::copy_n(scan.begin(), count, timed);
            } else {
                std::transform(scan.begin(), scan.begin() + count, nodes,
                               [](const TimedHqNode& sample) { return sample.node; });
            }
            return SL_RESULT_OK;
        }

    private:
        int _finishCurrentScanAndSwap_locked() {
            _scan_node_available_id = _getOperationBufferID_locked();
            int newOperationalID  =  1 - _scan_node_available_id;

            _scanbuffer[newOperationalID].clear();
            return newOperationalID;
        }

        int _getOperationBufferID_locked() {
            if (_scan_node_available_id < 0) return 0;
            return 1 - _scan_node_available_id;
        }

        std::vector<TimedHqNode>& _getOperationalBuffer_locked()
        {
            return _scanbuffer[_getOperationBufferID_locked()];
        }

        rp::hal::Locker _locker;
        rp::hal::Event  _data_waiter;

        _u64   _scan_begin_timestamp_uS[2];
        size_t _scan_node_buffer_size;
        int    _scan_node_available_id;
        std::atomic<bool>   _new_scan_ready;

        std::vector<TimedHqNode> _scanbuffer[2];
    };
}  // namespace sl
