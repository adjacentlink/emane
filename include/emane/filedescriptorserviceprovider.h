/*
 * Copyright (c) 2016 - Adjacent Link LLC, Bridgewater, New
 * Jersey
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of Adjacent Link LLC nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef EMANEFILEDESCRIPTORSERVICEPROVIDER_HEADER_
#define EMANEFILEDESCRIPTORSERVICEPROVIDER_HEADER_

#include <functional>

namespace EMANE
{
  /**
   * @class FileDescriptorServiceProvider
   *
   * @brief File Descriptor service interface allows for adding
   * arbitrary file descriptors for read or write processing on the
   * NEMQueuedLayer functor queue
   */
  class FileDescriptorServiceProvider
  {
  public:
    virtual ~FileDescriptorServiceProvider(){};

    enum class DescriptorType
    {
      /** Process when data is ready to read */
      READ,
      /** Process when data is ready to write */
        WRITE, 
        };

    /**
     * Adds a file descriptor for processing
     *
     * @param iFd File descriptor
     * @param type Type of descriptor processing 
     * @param fn A callable object
     */
    template <typename Function>
    void  addFileDescriptor(int iFd,
                            DescriptorType type,
                            Function fn);

    /**
     * Removed a file descriptor from processing
     *
     * @param  iFd File descriptor
     */
    virtual void removeFileDescriptor(int iFd) = 0;

  protected:
    FileDescriptorServiceProvider(){};

    using Callback = std::function<void(int iFd)>;

    virtual void addFileDescriptor_i(int iFd,
                                     DescriptorType type,
                                     Callback callback) = 0;
  };
}

#include "emane/filedescriptorserviceprovider.inl"

#endif // EMANEFILEDESCRIPTORSERVICEPROVIDER_HEADER_

/**
 * @page FileDescriptorService File Descriptor Service
 *
 * The @ref EMANE::FileDescriptorServiceProvider "FileDescriptorServiceProvider" is used by NEM
 * layer components to register file descriptors for servicing by the NEM layer functor queue.
 *
 * @section RegisteringATFileDescriptor Registering a File Descriptor
 *
 * @ref EMANE::FileDescriptorServiceProvider::addFileDescriptor
 * "FileDescriptorServiceProvider::addFileDescriptor" is a template method that associates a
 * callable and a type, either read or write, with a file descriptor.
 *
 * @snippet src/transports/virtual/virtualtransport.cc filedescriptorservice-registerfd-snippet
 *
 * The @ref EMANE::FileDescriptorServiceProvider "FileDescriptorServiceProvider" is accessed via the
 * @ref EMANE::PlatformServiceProvider "PlatformServiceProvider". All components are given a reference
 * to the @ref EMANE::PlatformServiceProvider "PlatformServiceProvider" when they are constructed.
 *
 * @section ServicingAFileDescriptor Servicing A File Descriptor
 *
 * The @ref EMANE::NEMQueuedLayer "NEMQueuedLayer" uses epoll with level-triggered events to service
 * registered file descriptors using their associated callables.
 *
 * @section UnregisteringATFileDescriptor Unregistering a File Descriptor
 *
 * A file descriptor can be removed from processing by using 
 * @ref EMANE::FileDescriptorServiceProvider::removeFileDescriptor
 * "FileDescriptorServiceProvider::removeFileDescriptor."
 *
 * @snippet src/transports/virtual/virtualtransport.cc filedescriptorservice-unregisterfd-snippet
 */
