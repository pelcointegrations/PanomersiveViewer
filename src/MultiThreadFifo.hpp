//============================================================================
// Copyright (c) 2012-2016 Pelco. All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_MULTI_THREAD_FIFO_H__
#define __PELCO_MULTI_THREAD_FIFO_H__

#include <mutex>
#include <condition_variable>
#include <chrono>
#include <queue>

//
// mt-safe fifo, simultanouely usable by multiple readers and multiple
// writers.  Timeouts used in this class are all in milliseconds,
// where a negative value indicates 'forever', and a 0 value indicates
// non-blocking.
//
template <class T>
class MultiThreadFifo {
    public:
        MultiThreadFifo(const MultiThreadFifo<T>& rhs) = delete;
        MultiThreadFifo<T>& operator=(const MultiThreadFifo<T>& rhs) = delete;

        // capacity == -1 indicates unbounded.  Any positive number
        // indicates a fixed maximum length, which could result in
        // block-on-write behavior
        MultiThreadFifo(int capacity = -1)
            : _capacity(capacity),
              _queueEmpty(true),
              _unblocked(false)
        {
        }

        bool willWriteBlock() const {
            std::unique_lock<std::mutex> lock(_mutex);
            return (_capacity >= 0) && (!_unblocked) &&
                (int(_queue.size()) == _capacity);
        }

        // returns false if the queue was full and could not be
        // written to within the given timeout period, or if the
        // fifo has been unblocked
        bool write(const T& val, int timeout = 0) {
            bool retVal = false;
            // wait for available space and grab the lock
            if (waitForRoomToWrite(timeout)) {
                // successful call to waitForRoomToWrite leaves
                // _mutex locked
                _queue.push(val);
                _queueEmpty = false;
                _dataAvailable.notify_one();
                _mutex.unlock();
                retVal = true;
            }
            return retVal;
        }

        // facility typically used in shutting down blocked threads.
        // After calling unblock, all current or future reads and
        // writes will return without blocking (with failure in
        // most cases).  Call reset() to resume normal blocking
        // behavior
        void unblock() {
            _unblocked = true;
            // wake up anything that might be currently blocked
            _dataAvailable.notify_all();
            _spaceAvailable.notify_all();
        }

        void reset() {
            _unblocked = false;
        }

        bool willReadBlock() const {
            return _queueEmpty;
        }

        // returns false if data was not available within the
        // timeout period, or if it was unblocked by a call to
        // unblockRead
        bool read(T& val, int timeout = -1) {
            bool retVal = false;
            if (waitForData(timeout)) {
                // successful waitForData leaves _mutex locked
                val = _queue.front();
                _queue.pop();
                _queueEmpty = _queue.empty();
                _spaceAvailable.notify_one();
                _mutex.unlock();
                retVal = true;
            }
            return retVal;
        }

    private:
        // wait for up to timeout (< 0 means infinite) for writing space
        // to become available.  Leave _mutex locked on successful
        // return
        bool waitForRoomToWrite(int timeout) {
            bool retVal = false;
            if (_capacity < 0) {
                // 'infinite' capacity -- always room to write
                _mutex.lock();
                retVal = true;
                // leave mutex locked on return
            } else {
                // finite capacity.  Make sure there's room to write
                auto waitFor = [this]() {
                    return (_unblocked) || (int(_queue.size()) < _capacity);
                };
                std::unique_lock<std::mutex> lock(_mutex);
                if (timeout < 0) {
                    // infinite timeout
                    _spaceAvailable.wait(lock, waitFor);
                } else {
                    // finite, absolute timeout
                    _spaceAvailable.wait_for(lock,
                            std::chrono::milliseconds(timeout),
                            waitFor);
                }
                retVal = (!_unblocked) &&
                    (int(_queue.size()) < _capacity);
                if (retVal) {
                    lock.release(); // leave mutex locked on return
                }
            }
            return retVal;
        }

        // wait for up to timeout (< 0 means infinite) for read data
        // to become available.  Leave _mutex locked on successful
        // return
        bool waitForData(int timeout) {
            bool retVal = false;
            if (!_unblocked) {
                // Special case the 0-timeout case with faster,
                // lock-free implementation
                if (timeout == 0) {
                    if (!_queueEmpty) {
                        _mutex.lock();
                        // we can definitively say there's no data
                        // outside the lock (doesn't matter if data
                        // shows up just after the check), but we
                        // can only definitively say that there
                        // _is_ data now that we've got the lock.
                        if (_queue.empty()) {
                            // only unlock if we failed
                            _mutex.unlock();
                        } else {
                            retVal = true;
                        }
                    }
                } else {
                    // Normal case, wait until there's data available
                    std::unique_lock<std::mutex> lock(_mutex);
                    if (_queue.empty()) {
                        auto waitFor = [this]() {
                            return (_unblocked) || (!_queue.empty());
                        };
                        if (timeout < 0) {
                            // infinite timeout
                            _dataAvailable.wait(lock, waitFor);
                        } else {
                            // finite, absolute timeout
                            _dataAvailable.wait_for(lock,
                                    std::chrono::milliseconds(timeout),
                                    waitFor);
                        }
                    }

                    retVal = (!_unblocked) && (!_queue.empty());
                    if (retVal) {
                        // leave _mutex locked when we return
                        lock.release();
                    }
                }
            }
            return retVal;
        }

        mutable std::mutex _mutex;
        const int _capacity;
        std::queue<T> _queue;
        // small optimization for non-blocking read
        volatile bool _queueEmpty;
        // unblocks read
        std::condition_variable _dataAvailable;
        // unblocks write
        std::condition_variable _spaceAvailable;

        // set to true by unblockRead or unblockWrite.
        // reads and writes will fail with this set, but they
        // also won't block
        volatile bool _unblocked;
};

#endif // __PELCO_MULTI_THREAD_FIFO_H__

