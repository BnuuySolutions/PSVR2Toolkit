#pragma once

#define WIN32_LEAN_AND_MEAN
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
      OverlappedWithData(AsyncFileWriter* asyncWriter)
        : overlapped()
        , writerInstance(asyncWriter)
        , hFile(NULL)
        , hTimer(NULL)
        , timeoutCallback(nullptr)
      {
        hTimer = CreateWaitableTimerW(NULL, TRUE, NULL);

        if (hTimer == NULL) {
          // If we can't create a timer (should be very unlikely), abort.
          abort();
        }
      }

      OVERLAPPED overlapped; // The OVERLAPPED structure for async I/O.
      AsyncFileWriter* writerInstance; // Pointer back to the class instance.
      HANDLE hFile; // Handle to the file being written to.
      HANDLE hTimer; // Handle to the waitable timer.
      std::function<void()> timeoutCallback; // Function to call on timeout.
    };

    // A simple thread-safe object pool for OverlappedWithData structures.
    class OverlappedDataPool {
    private:
      std::stack<OverlappedWithData*> m_pool;
      std::mutex m_mutex;
      AsyncFileWriter* m_asyncWriter;

    public:
      OverlappedDataPool(AsyncFileWriter* asyncWriter, size_t initialSize) {
        m_asyncWriter = asyncWriter;

        for (size_t i = 0; i < initialSize; ++i) {
          m_pool.push(new OverlappedWithData(asyncWriter));
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
          return new OverlappedWithData(m_asyncWriter);
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
      : m_pool(this, initialPoolSize) {
    }

    /**
     * @brief Destructor. Waits for all pending I/O operations to complete.
     * Note: This destructor does NOT close the file handle.
     */
    ~AsyncFileWriter() {
      // Wait for all pending operations to finish or timeout before the object is destroyed.
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
     * @param timeoutMilliseconds The timeout in milliseconds.
     * @return TRUE if the operation was successfully queued, otherwise FALSE.
     */
    BOOL Write(HANDLE hFile, LPCVOID buffer, DWORD numberOfBytesToWrite, DWORD timeoutMilliseconds, std::function<void()> timeoutCallback) {
      // Acquire an OverlappedWithData structure from our pool.
      OverlappedWithData* opData = m_pool.Acquire();
      // It's crucial to zero out the overlapped structure before each use.
      ZeroMemory(&opData->overlapped, sizeof(OverlappedWithData::overlapped));

      opData->hFile = hFile; // Store the file handle
      opData->timeoutCallback = timeoutCallback;

      pendingOperations++;

      if (!WriteFileEx(hFile, buffer, numberOfBytesToWrite, &opData->overlapped, FileWriteCompletionRoutine)) {
        // If the write fails to queue, release the object back to the pool.
        m_pool.Release(opData);
        pendingOperations--;
        return FALSE;
      }

      LARGE_INTEGER dueTime;

      // Timeout is negative for a relative time. Units are in 100-nanosecond intervals.
      dueTime.QuadPart = -static_cast<LONGLONG>(timeoutMilliseconds) * 10000LL;

      if (!SetWaitableTimer(opData->hTimer, &dueTime, 0, TimerTimeoutRoutine, opData, FALSE)) {
        // If setting the timer fails, we must cancel the I/O we just queued
        CancelIoEx(hFile, &opData->overlapped);
        // The FileWriteCompletionRoutine will still run and handle cleanup.
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
     * @brief The static completion routine for the waitable timer.
     * This is called if the write operation takes too long.
     */
    static VOID CALLBACK TimerTimeoutRoutine(LPVOID lpArgToCompletionRoutine, DWORD dwTimerLowValue, DWORD dwTimerHighValue) {
      OverlappedWithData* opData = static_cast<OverlappedWithData*>(lpArgToCompletionRoutine);

      // The timer has expired, so try to cancel the I/O operation.
      // Note: The FileWriteCompletionRoutine will still be called, but with an error code.
      CancelIoEx(opData->hFile, &opData->overlapped);

      // Call the user-provided timeout callback if it exists.
      if (opData->timeoutCallback) {
        opData->timeoutCallback();
      }
    }

    /**
     * @brief The static completion routine required by WriteFileEx.
     */
    static VOID CALLBACK FileWriteCompletionRoutine(
      DWORD dwErrorCode,
      DWORD dwNumberOfBytesTransfered,
      LPOVERLAPPED lpOverlapped)
    {
      OverlappedWithData* opData = reinterpret_cast<OverlappedWithData*>(lpOverlapped);

      if (opData->hTimer) {
        // We must cancel the timer in case the I/O completed *before* the timeout.
        // This prevents the TimerTimeoutRoutine from being called unnecessarily.
        CancelWaitableTimer(opData->hTimer);
      }

      if (opData->writerInstance) {
        opData->writerInstance->m_pool.Release(opData);
      }

      pendingOperations--;
    }
  };
}