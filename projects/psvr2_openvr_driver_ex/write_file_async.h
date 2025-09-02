#pragma once

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include <windows.h>
#include <functional>
#include <vector>
#include <atomic>
#include <mutex>
#include <stack>

#include "util.h"

namespace psvr2_toolkit {
    // A convenience wrapper class for performing asynchronous file writes on an existing handle.
    class AsyncFileWriter {
    private:
        // Structure to hold per-operation data.
        struct OverlappedWithData {
            OVERLAPPED overlapped;
            LARGE_INTEGER startTime;
            AsyncFileWriter* writerInstance; // Pointer back to the class instance.
        };

        // A simple thread-safe object pool for OverlappedWithData structures.
        class OverlappedDataPool {
        private:
            std::stack<OverlappedWithData*> m_pool;
            std::mutex m_mutex;

        public:
            OverlappedDataPool(size_t initialSize) {
                for (size_t i = 0; i < initialSize; ++i) {
                    m_pool.push(new OverlappedWithData());
                }
            }

            ~OverlappedDataPool() {
                std::scoped_lock<std::mutex> lock(m_mutex);
                while (!m_pool.empty()) {
                    delete m_pool.top();
                    m_pool.pop();
                }
            }

            OverlappedWithData* Acquire() {
                std::scoped_lock<std::mutex> lock(m_mutex);
                if (m_pool.empty()) {
                    // Pool is empty, allocate a new one on demand.
                    return new OverlappedWithData();
                }
                else {
                    OverlappedWithData* ptr = m_pool.top();
                    m_pool.pop();
                    return ptr;
                }
            }

            void Release(OverlappedWithData* ptr) {
                std::scoped_lock<std::mutex> lock(m_mutex);
                m_pool.push(ptr);
            }
        };

    public:
        /**
         * @brief Constructs the AsyncFileWriter.
         * @param callback The function to be called upon completion of each write operation.
         * @param initialPoolSize The number of OverlappedWithData objects to pre-allocate.
         */
        AsyncFileWriter(size_t initialPoolSize = 16)
            : m_pool(initialPoolSize) {
        }

        /**
         * @brief Destructor. Waits for all pending I/O operations to complete.
         * Note: This destructor does NOT close the file handle.
         */
        ~AsyncFileWriter() {
            // Wait for all pending operations to finish before the object is destroyed.
            while (pendingOperations > 0) {
                SleepEx(100, TRUE); // Sleep in an alertable state to process callbacks.
            }
        }

        // Disable copy constructor and assignment operator.
        AsyncFileWriter(const AsyncFileWriter&) = delete;
        AsyncFileWriter& operator=(const AsyncFileWriter&) = delete;

        /**
         * @brief Initiates an asynchronous write operation.
         * @param buffer Pointer to the buffer containing the data to be written.
         * @param numberOfBytesToWrite The number of bytes to be written to the file.
         * @return TRUE if the operation was successfully queued, otherwise FALSE.
         */
        BOOL Write(HANDLE hFile, LPCVOID buffer, DWORD numberOfBytesToWrite) {
            // Acquire an OverlappedWithData structure from our pool.
            OverlappedWithData* opData = m_pool.Acquire();
            // It's crucial to zero out the structure before each use.
            ZeroMemory(opData, sizeof(OverlappedWithData));

            opData->writerInstance = this;
            QueryPerformanceCounter(&opData->startTime);

            pendingOperations++;

            if (!WriteFileEx(hFile, buffer, numberOfBytesToWrite, &opData->overlapped, FileWriteCompletionRoutine)) {
                // If the write fails to queue, release the object back to the pool.
                m_pool.Release(opData);
                pendingOperations--;
                return FALSE;
            }

            return TRUE;
        }

        long GetPendingOperationCount()
        {
            return pendingOperations;
        }

    private:
        OverlappedDataPool m_pool;
        static inline std::atomic<long> pendingOperations = 0;

        /**
         * @brief The static completion routine required by WriteFileEx.
         */
        static VOID CALLBACK FileWriteCompletionRoutine(
            DWORD dwErrorCode,
            DWORD dwNumberOfBytesTransfered,
            LPOVERLAPPED lpOverlapped)
        {
            OverlappedWithData* opData = reinterpret_cast<OverlappedWithData*>(lpOverlapped);

            LARGE_INTEGER endTime, frequency;
            QueryPerformanceFrequency(&frequency);
            QueryPerformanceCounter(&endTime);

            double elapsedTime = static_cast<double>(endTime.QuadPart - opData->startTime.QuadPart)
                / static_cast<double>(frequency.QuadPart);

            if (opData->writerInstance) {
                opData->writerInstance->m_pool.Release(opData);
            }

            pendingOperations--;
        }
    };
}