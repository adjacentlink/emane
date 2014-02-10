/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
 * Copyright (c) 2008 - DRS CenGen, LLC, Columbia, Maryland
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
 * * Neither the name of DRS CenGen, LLC nor the names of its
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

#ifndef EMANEEXCEPTION_HEADER_
#define EMANEEXCEPTION_HEADER_

#include <exception>
#include <string>
#include <cstdarg>

namespace EMANE
{
  /**
   * @class Exception
   *
   * @brief Exception base class that allows for type and 
   * description information.
   */
  class Exception : public std::exception
  {
  public:
    /**
     * Destroys an instance
     */
    ~Exception() throw(){}
    
    /**
     * Gets the exception description
     *
     * @return Null terminated description
     */
    const char* what() const throw()
    {
      return sDescription_.c_str();
    }
    
    /**
     * Gets the exception type name
     *
     * @return Null terminated type name
     */
    virtual const char * type() const throw()
    {
      return sType_.c_str();
    }

  protected:
    Exception(const std::string & sType,
              const std::string & sDescription = {}):
      sType_{sType},
      sDescription_{sDescription}{}
    
  private:
    const std::string sType_;
    const std::string sDescription_;
  };
  
  /**
   * Makes an exception using a printf style description
   *
   * @tparam T Type derived from Exception 
   * @tparam len Maximum length of message
   *
   * @param fmt format string (see printf)
   * @param ... Variable data (see printf)
   *
   * @return Newly created exception
   */
  template<typename T, std::size_t len = 1024>
  T makeException(const char *fmt,...)
  {
    char buf[len];

    va_list ap;
    
    va_start(ap, fmt);
      
    vsnprintf(buf, sizeof(buf),fmt,ap);

    va_end(ap);
    
    return T{buf};
  }
}

#endif // EMANEEXCEPTION_HEADER_
