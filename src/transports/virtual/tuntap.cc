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

#include <ace/SOCK_Dgram.h>
#include <ace/Handle_Set.h>
#include <ace/INET_Addr.h>
#include <ace/OS_NS_stdio.h>
#include <ace/OS_NS_unistd.h>
#include <ace/OS_NS_sys_uio.h>
#include <ace/OS_NS_sys_socket.h>
#include <ace/OS_NS_fcntl.h>
#include <ace/OS_NS_arpa_inet.h>

#include "tuntap.h"


/*
  Create device node: mknod /dev/net/tun c 10 200
  Add following line to the /etc/modules.conf: alias char-major-10-200 tun
  Run: depmod -a 
  Driver will be automatically loaded when application access /dev/net/tun.
*/


/**
 *
 * @class TunTap
 *
 * @brief Tuntap networkdevice service provider.
 *
 */
EMANE::Transports::Virtual::TunTap::TunTap (PlatformServiceProvider * pPlatformService) : 
 pPlatformService_(pPlatformService),
 tunHandle_(ACE_INVALID_HANDLE), 
 tunName_(""), 
 tunPath_(""), 
 tunIndex_(-1)
{ }


/**
 *
 * @brief destructor
 *
 */
EMANE::Transports::Virtual::TunTap::~TunTap ()
{ }



int
EMANE::Transports::Virtual::TunTap::open (const char *sDevicePath, const char *sDeviceName)
{
  int result;

#if defined( __APPLE__)

  if ((tunHandle_ = ACE_OS::open (sDevicePath, O_RDWR)) == ACE_INVALID_HANDLE) {
    LOGGER_STANDARD_LOGGING(pPlatformService_, 
                            ABORT_LEVEL, 
                            "TunTap::%s:open:error %s", 
                            __func__, 
                            ACE_OS::strerror (errno));

    // fail
    return -1;
  }
   // save the dev path, name, guid and index
  tunPath_  = sDevicePath;
  tunName_  = sDeviceName;
  tunGuid_  = "n/a";

  // success
  result = 0;

#elif defined(__linux__)

  // open tuntap device
  if ((tunHandle_ = ACE_OS::open (sDevicePath, O_RDWR)) == ACE_INVALID_HANDLE) {
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                            ABORT_LEVEL, 
                            "TunTap::%s:open:error %s", 
                            __func__, 
                            ACE_OS::strerror (errno));

    // fail
    return -1;
  }

  // interface info
  struct ifreq ifr;

  // clear ifr
  ACE_OS::memset (&ifr, 0, sizeof (ifr));

  // copy dev name
  ACE_OS::strncpy (ifr.ifr_name, sDeviceName, sizeof (ifr.ifr_name));

  // set flags no proto info and tap mode
  ifr.ifr_flags = IFF_NO_PI | IFF_TAP;

  // set tun flags
  if (ACE_OS::ioctl (tunHandle_, TUNSETIFF, &ifr) < 0) {
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                            ABORT_LEVEL, 
                            "TunTap::%s:ioctl:error %s", 
                            __func__, 
                            ACE_OS::strerror (errno));

    // fail
    return -1;
  }

  // clear ifr
  ACE_OS::memset (&ifr, 0, sizeof (ifr));

  // copy dev name
  ACE_OS::strncpy (ifr.ifr_name, sDeviceName, sizeof (ifr.ifr_name));

  // open ipv4 control socket
  ACE_SOCK_Dgram ctrlsock (ACE_INET_Addr ("0.0.0.0:0", AF_INET));

  // get iff index
  if (ACE_OS::ioctl (ctrlsock.get_handle (), SIOCGIFINDEX, &ifr) < 0) {
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                            ABORT_LEVEL, 
                            "TunTap::%s:getindex:error %s", 
                            __func__, 
                            ACE_OS::strerror (errno));

    // fail
    return -1;
  }

  // save the dev path, name, guid and index
  tunPath_  = sDevicePath;
  tunName_  = sDeviceName;
  tunGuid_  = "n/a";
  tunIndex_ = ifr.ifr_ifindex;

  // close control socket
  ctrlsock.close();

  // success
  result = 0;

#elif defined(WIN32)

  HKEY networkConnectionsKey;
  HKEY networkControlKey;
  LONG status;

  // open the network connections key
  status = RegOpenKeyEx(
                        HKEY_LOCAL_MACHINE,
                        sDevicePath,
                        0,
                        KEY_READ,
                        &networkConnectionsKey);

  // error
  if (status != ERROR_SUCCESS) {
    LOGGER_STANDARD_LOGGING(pPlatformService_, 
                            ABORT_LEVEL, 
                            "TunTap::%s, RegOpenKeyEx:%s:error %s", 
                            __func__, 
                            sDevicePath, 
                            ACE_OS::strerror(errno));

    // fail
    return -1;
  }

  // device status flag
  bool bDeviceEnabled = false;

  // check enums until handle obtained, end of list or failure
  for(int index = 0; tunHandle_ == ACE_INVALID_HANDLE; ++index) {
    // connection key
    HKEY connectionKey;

    // enum name
    char sEnumName[256];

    // enum name len
    DWORD enumNameLen = sizeof (sEnumName);

    // search through each enum in the network connections key
    status = RegEnumKeyEx(networkConnectionsKey, index, sEnumName, &enumNameLen, NULL, NULL, NULL, NULL);

    // end of list
    if (status == ERROR_NO_MORE_ITEMS) {
      LOGGER_STANDARD_LOGGING(pPlatformService_, 
                              ABORT_LEVEL, 
                              "TunTap::%s, ReqEnumKeyEx:%s:error %s", 
                              __func__, 
                              sEnumName, 
                              ACE_OS::strerror(errno));

      // fail
      return -1;
    }
    // error
    else if (status != ERROR_SUCCESS) {
      LOGGER_STANDARD_LOGGING(pPlatformService_, 
                              ABORT_LEVEL, 
                              "TunTap::%s, ReqEnumKeyEx:%s:error %s", 
                              __func__, 
                              sEnumName, 
                              ACE_OS::strerror(errno));

      // fail
      return -1;
    }
    else {
      // the connection
      char sConnectionPath[256];

      // format the connection path
      ACE_OS::snprintf (sConnectionPath, sizeof(sConnectionPath), "%s\\%s\\Connection", sDevicePath, sEnumName);

      // open connection key
      status = RegOpenKeyEx(HKEY_LOCAL_MACHINE, sConnectionPath, 0, KEY_READ, &connectionKey); 

      if (status == ERROR_SUCCESS) {
        // the connection name
        char sConnectionName[256];

        // the connection name len
        DWORD connectionLen = sizeof (sConnectionName);
        DWORD dataType;

        // query name from the connection
        status = RegQueryValueEx(connectionKey, "Name", NULL, &dataType, (BYTE*)sConnectionName, &connectionLen);

        // success 
        if ((status == ERROR_SUCCESS) && (dataType == REG_SZ)) {

         // is this the target device
         if (ACE_OS::strcmp (sConnectionName, sDeviceName) == 0) {
            // target path
            char sTargetPath[256];
      
            // format the device path
            ACE_OS::snprintf (sTargetPath, sizeof(sTargetPath), "%s%s%s", "\\\\.\\Global\\", sEnumName, ".tap");

            // get the tap handle
            tunHandle_ = CreateFile (sTargetPath, 
                                     GENERIC_WRITE | GENERIC_READ, 
                                     0, 
                                     0, 
                                     OPEN_EXISTING, 
                                     FILE_ATTRIBUTE_SYSTEM | FILE_FLAG_OVERLAPPED, 
                                     0);

            // good handle
            if (tunHandle_ != ACE_INVALID_HANDLE) {

              // clear write overlapped io
              memset(&ioWrite_.overlapped, 0x0, sizeof(ioWrite_.overlapped));

              // clear read overlapped io
              memset(&ioRead_.overlapped,  0x0, sizeof(ioRead_.overlapped));

              // create overlapped read event
              if ((ioRead_.overlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL)) == NULL) {
                LOGGER_STANDARD_LOGGING(pPlatformService_, 
                                        ABORT_LEVEL, 
                                        "TunTap::%s, CreateEvent:error %s", 
                                        __func__,
                                        ACE_OS::strerror(errno));

                // fail
                return -1;
              }

              // save the dev path, name, and guid
              tunPath_  = sTargetPath;
              tunName_  = sDeviceName;
              tunGuid_  = sEnumName;
            }
            else {
              LOGGER_STANDARD_LOGGING(pPlatformService_, 
                                      ABORT_LEVEL, 
                                      "TunTap::%s:error %s", 
                                      __func__, 
                                      ACE_OS::strerror(errno));

              // fail
              return -1;
            }
          }
        }
      }
    }
    // close the connection key
    RegCloseKey (connectionKey);
  }

  // close the network connections key 
  RegCloseKey (networkConnectionsKey);

  // open the network control key
  status = RegOpenKeyEx(
                        HKEY_LOCAL_MACHINE,
                        NETWORK_CONTROL_PATH,
                        0,
                        KEY_READ,
                        &networkControlKey);

  // error
  if (status != ERROR_SUCCESS) {
    LOGGER_STANDARD_LOGGING(pPlatformService_, 
                            ABORT_LEVEL, 
                            "TunTap::%s, RegOpenKeyEx:%s:error %s", 
                            __func__, 
                            NETWORK_CONTROL_PATH, 
                            ACE_OS::strerror(errno));

    // fail
    return -1;
  }

  // check enums until end of list or fail
  for(int index = 0; ;++index) {

    // control key
    HKEY controlKey;

    // enum name
    char sEnumName[256];

    // enum name len
    DWORD enumNameLen = sizeof (sEnumName);

    // search through each enum in the network control key
    status = RegEnumKeyEx(networkControlKey, index, sEnumName, &enumNameLen, NULL, NULL, NULL, NULL);

    // end of list
    if (status == ERROR_NO_MORE_ITEMS) {
      LOGGER_STANDARD_LOGGING(pPlatformService_, 
                              ABORT_LEVEL, 
                              "TunTap::%s, ReqEnumKeyEx:%s:error %s", 
                              __func__, 
                              sEnumName, 
                              ACE_OS::strerror(errno));

      // fail
      return -1;
    }
    // error
    else if (status != ERROR_SUCCESS) {
      LOGGER_STANDARD_LOGGING(pPlatformService_, 
                              ABORT_LEVEL, 
                              "TunTap::%s, ReqEnumKeyEx:%s:error %s", 
                              __func__, 
                              sEnumName, 
                              ACE_OS::strerror(errno));

      // fail
      return -1;
    }
    else {
      // the control
      char sControlPath[256];

      // format the control path
      ACE_OS::snprintf (sControlPath, sizeof(sControlPath), "%s\\%s", NETWORK_CONTROL_PATH, sEnumName);

      // open control key
      status = RegOpenKeyEx(HKEY_LOCAL_MACHINE, sControlPath, 0, KEY_READ, &controlKey); 

      if (status == ERROR_SUCCESS) {
        // the instance id name
        char sInstanceId[256];

        // instance id len
        DWORD instanceIdLen = sizeof (sInstanceId);

        // data type
        DWORD dataType;

        // query Name from the connection
        status = RegQueryValueEx(controlKey, "NetCfgInstanceId", NULL, &dataType, (BYTE*)sInstanceId, &instanceIdLen);

        // success 
        if (status == ERROR_SUCCESS && dataType == REG_SZ) {
          // is this the target instance
          if (ACE_OS::strcmp (sInstanceId, tunGuid_.c_str()) == 0) {

             // save the dev index
             tunIndex_ = index;

             // done
             break;
          }
        }
      }
    }
    // close the control key
    RegCloseKey (controlKey);
  }

  // close the network control key 
  RegCloseKey (networkControlKey);

  // success
  result = 0;

#else
#error "unknown platfrom"
#endif


  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "TunTap::%s, path %s, name %s, guid %s, index %d",
                          __func__, 
                          tunPath_.c_str(), 
                          tunName_.c_str(), 
                          tunGuid_.c_str(),
                          tunIndex_);

  // return result
  return result;
}


int EMANE::Transports::Virtual::TunTap::close ()
{
  ACE_OS::close (tunHandle_);

#ifdef WIN32

  ACE_OS::socket_fini();

#endif

  // return 0
  return 0;
}


ACE_HANDLE EMANE::Transports::Virtual::TunTap::get_handle ()
{
  return tunHandle_;
}


int
EMANE::Transports::Virtual::TunTap::activate (bool arpEnabled)
{
#if defined(__linux__) || defined(__APPLE__)

  // set if flags
  int flags = IFF_UP;

  if(arpEnabled == false) {
    flags |= IFF_NOARP;
  }

  return set_flags (flags, 1);

#elif defined(WIN32)

  ACE_UNUSED_ARG(arpEnabled);

  // start win32 sockets
  int err = ACE_OS::socket_init(2, 2);

  if(err == 0) {
    ULONG status = TRUE;
    DWORD len;

    // start device
    if (!DeviceIoControl(tunHandle_, 
                         TAP_SET_MEDIA_STATUS, 
                         &status, 
                         sizeof (status), 
                         &status, 
                         sizeof (status), 
                         &len, 
                         NULL)) {

      LOGGER_STANDARD_LOGGING(pPlatformService_, 
                              ABORT_LEVEL, 
                              "TunTap::%s:ioctl:error %s", 
                              __func__, 
                              ACE_OS::strerror (errno));

      // fail
      return -1;
    }
    else {
      // success
      return 0;
    }
  }
  else {
     // fail
     return -1;
  }

#else
#error "unknown platfrom"
#endif

}


int
EMANE::Transports::Virtual::TunTap::deactivate ()
{
#if defined(__linux__) || defined(__APPLE__)

  return set_flags (IFF_UP, -1);

#elif defined(WIN32)

  if (!CancelIo (tunHandle_)) {
    LOGGER_STANDARD_LOGGING(pPlatformService_, 
                            ABORT_LEVEL, 
                            "TunTap::%s:ioctl:error %s", 
                            __func__, 
                            ACE_OS::strerror (errno));

    // fail
    return -1;
  }
  else {
    // success
    return 0;
  }

#else
#error "unknown platfrom"
#endif
}


int
EMANE::Transports::Virtual::TunTap::set_addr (const ACE_INET_Addr & addr, const ACE_INET_Addr & mask)
{
  // save of copy of the addr and mask in string format
  std::string sAddress = addr.get_host_addr ();
  std::string sNetMask = mask.get_host_addr ();

  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                          DEBUG_LEVEL, 
                          "TunTap::%s, type %s, addr %s, mask %s", __func__, 
                          addr.get_type() == AF_INET  ? "ipv4" :
                          addr.get_type() == AF_INET6 ? "ipv6" : "?",
                          sAddress.c_str(), 
                          sNetMask.c_str());

#ifdef __linux__
  // addr type ipv4
  if(addr.get_type() == AF_INET) {
      // interface info
      struct ifreq ifr;

      // clear ifr
      ACE_OS::memset (&ifr, 0, sizeof (ifr));

      // copy dev name
      ACE_OS::strncpy (ifr.ifr_name, tunName_.c_str (), sizeof (ifr.ifr_name));

      // copy addr and family
      ((struct sockaddr_in_t *) &ifr.ifr_addr)->sin_family = AF_INET;
      ((struct sockaddr_in_t *) &ifr.ifr_addr)->sin_addr.s_addr = ACE_HTONL (addr.get_ip_address ());

      // open ipv4 control socket
      ACE_SOCK_Dgram ctrlsock (ACE_INET_Addr ("0.0.0.0:0", AF_INET));

      // set addr
      if (ACE_OS::ioctl (ctrlsock.get_handle (), SIOCSIFADDR, &ifr) < 0) {
        LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                ABORT_LEVEL, 
                                "TunTap::%s:setaddr:error %s", 
                                __func__, 
                                ACE_OS::strerror (errno));
        return -1;
      }
      else {
        // save tun addr
        tunAddr_ = addr;
      }

      // clear ifr
      ACE_OS::memset (&ifr, 0, sizeof (ifr));

      // copy dev name
      ACE_OS::strncpy (ifr.ifr_name, tunName_.c_str (), sizeof (ifr.ifr_name));

      // copy addr and family
      ((struct sockaddr_in_t *) &ifr.ifr_addr)->sin_family = AF_INET;
      ((struct sockaddr_in_t *) &ifr.ifr_addr)->sin_addr.s_addr = ACE_HTONL (mask.get_ip_address ());

      // set netmask
      if (ACE_OS::ioctl (ctrlsock.get_handle (), SIOCSIFNETMASK, &ifr) < 0) {
        LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                ABORT_LEVEL, 
                                "TunTap::%s:setmask:error %s", 
                                __func__, 
                                ACE_OS::strerror (errno));

        // fail
        return -1;
      }
      else {
        // save tun mask
        tunMask_ = mask;
      }

      // close control socket
      ctrlsock.close();
    }
  // addr type ipv6
  else if(addr.get_type() == AF_INET6) {
      // interface info ipv6
      struct in6_ifreq {
        struct in6_addr ifr6_addr;
        ACE_UINT32      ifr6_prefixlen;
        ACE_INT32       ifr6_ifindex;
      } ifr6;

      // ipv6 prefix
      struct in6_addr in6_prefix;

      // clear ifr6
      ACE_OS::memset (&ifr6, 0, sizeof (ifr6));

      // copy index
      ifr6.ifr6_ifindex = tunIndex_;

      // copy mask
      if(ACE_OS::inet_pton(AF_INET6, sNetMask.c_str(), &in6_prefix) < 0) {
        LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                ABORT_LEVEL, 
                                "TunTap::%s:copyprefix:error %s", 
                                __func__, 
                                ACE_OS::strerror (errno));
        return -1;
      }

      // copy prefix len
      ifr6.ifr6_prefixlen = Utils::get_prefixlen(&in6_prefix);

      // copy address
      if(ACE_OS::inet_pton(AF_INET6, sAddress.c_str(), &ifr6.ifr6_addr) < 0) {
        LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                ABORT_LEVEL, 
                                "TunTap::%s:copyaddr:error %s", 
                                __func__, 
                                ACE_OS::strerror (errno));
        return -1;
      }

      // open ipv6 control socket
      ACE_SOCK_Dgram ctrlsock (ACE_INET_Addr(0,"::", AF_INET6));

      // set addr
      if (ACE_OS::ioctl (ctrlsock.get_handle (), SIOCSIFADDR, &ifr6) < 0) {
        LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                ABORT_LEVEL, 
                                "TunTap::%s:setaddr:error %s", 
                                __func__, 
                                ACE_OS::strerror (errno));
        return -1;
      }
      else {
        // save tun addr
        tunAddr_ = addr;

        // save tun mask
        tunMask_ = mask;
      }

      // close control socket
      ctrlsock.close();
   }

#elif defined(__APPLE__)

  // TODO add ipv6 support

  struct ifaliasreq ifar;
  memset(&ifar, 0, sizeof(ifar));

  // addr
  struct sockaddr_in_t saddr;
  saddr.sin_family = AF_INET;
  saddr.sin_len = sizeof(struct sockaddr_in_t);
  saddr.sin_addr.s_addr = ACE_HTONL (addr.get_ip_address ());

  // mask
  struct sockaddr_in_t smask;
  smask.sin_family = AF_INET;
  smask.sin_len = sizeof(struct sockaddr_in_t);
  smask.sin_addr.s_addr = ACE_HTONL (mask.get_ip_address ());

  // copy addr
  memcpy(&ifar.ifra_addr, &saddr, sizeof(struct sockaddr_in_t));

  // copy mask
  memcpy(&ifar.ifra_mask, &smask, sizeof(struct sockaddr_in_t));

  // copy dev name
  ACE_OS::strncpy (ifar.ifra_name, tunName_.c_str (), sizeof (ifar.ifra_name));

  // open ipv4 control socket
  ACE_SOCK_Dgram ctrlsock (ACE_INET_Addr ("0.0.0.0:0", AF_INET));
 
  // set addr
  if (ACE_OS::ioctl (ctrlsock.get_handle (), SIOCAIFADDR, &ifar) < 0) {
    LOGGER_STANDARD_LOGGING(pPlatformService_, 
                            ABORT_LEVEL, 
                            "TunTap::%s:setaddr:error %s", 
                            __func__, 
                            ACE_OS::strerror (errno));

    // fail
    return -1;
  }
  else {
    // save tun addr
    tunAddr_ = addr;

    // save tun mask
    tunMask_ = mask;
  }

  // close control socket
  ctrlsock.close();


#elif defined(WIN32)

  // TODO add ipv6 support

  // command string
  char sCommandString[256];

  // format command string 
  snprintf (sCommandString, sizeof (sCommandString),
                          "netsh interface ip set address \"%s\" static %s %s",
                          tunName_.c_str(), sAddress.c_str(), sNetMask.c_str());

  if (system(sCommandString) != 0) {
    LOGGER_STANDARD_LOGGING(pPlatformService_, 
                            ABORT_LEVEL, 
                            "TunTap::%s:system:error %s", 
                            __func__, 
                            ACE_OS::strerror (errno));

    // fail
    return -1;
  }
  else {
    // save tun addr
    tunAddr_ = addr;

    // save tun mask
    tunMask_ = mask;
  }

#else
#error "unknown platfrom"
#endif

  // success
  return 0;
}


int EMANE::Transports::Virtual::TunTap::set_ethaddr (ACE_UINT16 id)
{
  // eth addr
  Utils::EtherAddr ethAddr;

  // locally administered 02:02:00:00:XX:XX
  ethAddr.words.word1 = ACE_HTONS(0x0202);
  ethAddr.words.word2 = ACE_HTONS(0x0000);
  ethAddr.words.word3 = ACE_HTONS(id);

  return set_ethaddr(ethAddr);
}

int EMANE::Transports::Virtual::TunTap::set_ethaddr (const Utils::EtherAddr & ethAddr)
{
  LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                            DEBUG_LEVEL, 
                            "TunTap::%s:%s", 
                            __func__, 
                            addr_to_string(&ethAddr));

#ifdef __linux__

    // interface info
    struct ifreq ifr;

    // clear ifr
    ACE_OS::memset (&ifr, 0, sizeof (ifr));

    // copy dev name
    ACE_OS::strncpy (ifr.ifr_name, tunName_.c_str (), sizeof (ifr.ifr_name));

    // copy hwaddr
    ACE_OS::memcpy (ifr.ifr_hwaddr.sa_data, &ethAddr, Utils::ETH_ALEN);

    // copy hwaddr family
    ifr.ifr_hwaddr.sa_family = Utils::ARPHRD_ETHER;

    // open ipv4 control socket
    ACE_SOCK_Dgram ctrlsock (ACE_INET_Addr ("0.0.0.0:0", AF_INET));

    // set hw addr
    if (ACE_OS::ioctl (ctrlsock.get_handle (), SIOCSIFHWADDR, &ifr) < 0) {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              ABORT_LEVEL, 
                              "TunTap::%s:sethwaddr:error %s", 
                              __func__, 
                              ACE_OS::strerror (errno));

      // fail
      return -1;
    }

    // close control socket
    ctrlsock.close();

#elif defined(__APPLE__)

    // interface info
    struct ifreq ifr;

    // clear ifr
    ACE_OS::memset (&ifr, 0, sizeof (ifr));

    // copy dev name
    ACE_OS::strncpy (ifr.ifr_name, tunName_.c_str (), sizeof (ifr.ifr_name));

    // copy hwaddr
    ACE_OS::memcpy (ifr.ifr_addr.sa_data, &ethAddr, Utils::ETH_ALEN);

    // copy hwaddr family
    ifr.ifr_addr.sa_family = AF_LINK;

    // copy hwaddr len
    ifr.ifr_addr.sa_len = Utils::ETH_ALEN;
    
    // open ipv4 control socket
    ACE_SOCK_Dgram ctrlsock (ACE_INET_Addr ("0.0.0.0:0", AF_INET));

    // set hw addr
    if (ACE_OS::ioctl (ctrlsock.get_handle (), SIOCSIFLLADDR, &ifr) < 0) {
      LOGGER_STANDARD_LOGGING(pPlatformService_, 
                              ABORT_LEVEL, 
                              "TunTap::%s:sethwaddr:error %s", 
                              __func__, 
                              ACE_OS::strerror (errno));

      // fail
      return -1;
    }

    // close control socket
    ctrlsock.close();

#elif defined(WIN32)

  LONG status;
  HKEY controlKey;
  char sTargetPath[256];

  // format the dev path
  ACE_OS::snprintf (sTargetPath, sizeof(sTargetPath), "%s\\%04d", NETWORK_CONTROL_PATH, tunIndex_);

  // open control key
  status = RegOpenKeyEx(HKEY_LOCAL_MACHINE, sTargetPath, 0, KEY_READ | KEY_WRITE, &controlKey); 

  // success
  if (status == ERROR_SUCCESS) {
    // the mac address
    char sMACAddr[256];

    // format the mac address
    DWORD macLen = ACE_OS::snprintf (sMACAddr, sizeof(sMACAddr), "%02X%02X%02X%02X%02X%02X",
                                            ethAddr.bytes.buff[0] & 0xFF,
                                            ethAddr.bytes.buff[1] & 0xFF,
                                            ethAddr.bytes.buff[2] & 0xFF,
                                            ethAddr.bytes.buff[3] & 0xFF,
                                            ethAddr.bytes.buff[4] & 0xFF,
                                            ethAddr.bytes.buff[5] & 0xFF);
   

    // set MAC
    status = RegSetValueEx(controlKey, "MAC", 0, REG_SZ, (BYTE*)sMACAddr, macLen);

    // error  
    if (status != ERROR_SUCCESS) {
       LOGGER_STANDARD_LOGGING(pPlatformService_, 
                               ABORT_LEVEL, 
                               "TunTap::%s:RegSetValueEx:error %s", 
                               __func__, 
                               ACE_OS::strerror (errno));

       // fail
       return -1;
    }
  }
  // error
  else {
    LOGGER_STANDARD_LOGGING(pPlatformService_, 
                            ABORT_LEVEL, 
                            "TunTap::%sRegOpenKeyEx:error %s", 
                            __func__, 
                            ACE_OS::strerror (errno));

    // fail
    return -1;
  }

  // close the control key
  RegCloseKey (controlKey);

#else
#error "unknown platfrom"
#endif

  // success
  return 0;
}



int EMANE::Transports::Virtual::TunTap::writev (const struct iovec *iov, size_t iov_len)
{
  int result;
  
#if defined(__linux__) || defined(__APPLE__)

  result = ACE_OS::writev (tunHandle_, iov, iov_len);

#elif defined(WIN32)

   // total copied
   size_t bytesCopied = 0;

   // room available
   size_t bytesAvailable = sizeof(ioWrite_.buffer);

   // for each iov 
   for(size_t i = 0; (i < iov_len) && (bytesAvailable > 0); ++i) {
     size_t bytes = iov[i].iov_len;

     // check for room available
     if(bytesAvailable <= bytes) {
       bytes = bytesAvailable;
     }

     // copy from buf to iov
     ACE_OS::memcpy(&ioWrite_.buffer[bytesCopied], iov[i].iov_base, bytes);

     // bump bytes copied
     bytesCopied += bytes;

     // subtract bytes available
     bytesAvailable -= bytes;
  }

  // set io write offsets
  ioWrite_.overlapped.Offset = 0;
  ioWrite_.overlapped.OffsetHigh = 0;

  // write to device
  bool status = WriteFile(tunHandle_, 
                           ioWrite_.buffer, 
                           bytesCopied, 
                           &ioWrite_.byteCount,
                           &ioWrite_.overlapped);

  // check status
  if (!status) {
     // fail
     result = -1;
  }
  else {
     // save result
     result = ioWrite_.byteCount;
  }

#else
#error "unknown platfrom"
#endif

  // check result
  if (result < 0) {
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                            ABORT_LEVEL, 
                            "TunTap::%s:write:error %s", 
                            __func__, 
                            ACE_OS::strerror (errno));
  }

  // return result
  return result;
}



int EMANE::Transports::Virtual::TunTap::readv (struct iovec *iov, size_t iov_len)
{
  int result;

#if defined(__linux__) || defined(__APPLE__)

  result = ACE_OS::readv (tunHandle_, iov, iov_len);

#elif defined(WIN32)

  ioRead_.overlapped.Offset = 0;
  ioRead_.overlapped.OffsetHigh = 0;
  ResetEvent (ioRead_.overlapped.hEvent);

  bool status = ReadFile(tunHandle_,
                      ioRead_.buffer,
                      (DWORD)sizeof(ioRead_.buffer),
                      &ioRead_.byteCount,
                      &ioRead_.overlapped);

  // check status
  if (!status) {
    if (GetLastError() == ERROR_IO_PENDING) {
      WaitForSingleObject(ioRead_.overlapped.hEvent, INFINITE);
      if (!GetOverlappedResult(tunHandle_, &ioRead_.overlapped, &ioRead_.byteCount, FALSE)) {
        LOGGER_STANDARD_LOGGING(pPlatformService_, 
                                ABORT_LEVEL, 
                                "TunTap::%s:read:error %s", 
                                __func__, 
                                ACE_OS::strerror (GetLastError()));

        // fail
        return -1;   
      }
    }
    // error
    else {
      LOGGER_STANDARD_LOGGING(pPlatformService_, 
                              ABORT_LEVEL, 
                              "TunTap::%s:read:error %s", 
                              __func__, 
                              ACE_OS::strerror (GetLastError()));

      // fail
      return -1;
    }
  }

  // total copied
  size_t bytesCopied = 0;

  // bytes to copy
  size_t bytesToCopy = ioRead_.byteCount;

  // for each iov 
  for(size_t i = 0; (i < iov_len) && (bytesToCopy > 0); ++i) {
    size_t bytes = bytesToCopy;

    // check for room available
    if(iov[i].iov_len <= bytes) {
      bytes = iov[i].iov_len;
    }

    // copy from buf to iov
    ACE_OS::memcpy(iov[i].iov_base, &ioRead_.buffer[bytesCopied], bytes);

    // bump bytes copied
    bytesCopied += bytes;

    // subtract bytes to copy
    bytesToCopy -= bytes;
  }

  // number of bytes copied
  result = bytesCopied;

#else
#error "unknown platfrom"
#endif

  // check result
  if (result < 0) {
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                            ABORT_LEVEL, 
                            "TunTap::%s:read:error %s", 
                            __func__, 
                            ACE_OS::strerror (errno));
  }

  // return result
  return result;
}


ACE_INET_Addr & EMANE::Transports::Virtual::TunTap::get_addr ()
{
  // return tun address
  return tunAddr_;
}


ACE_INET_Addr & EMANE::Transports::Virtual::TunTap::get_mask ()
{
  // return tun address
  return tunMask_;
}



#if defined(__linux__) || defined(__APPLE__)

// private methods
int EMANE::Transports::Virtual::TunTap::set_flags (int newflags, int cmd)
{
  // interface info
  struct ifreq ifr;

  // clear ifr
  ACE_OS::memset (&ifr, 0, sizeof (ifr));

  // copy dev name
  ACE_OS::strncpy (ifr.ifr_name, tunName_.c_str (), IFNAMSIZ);

  // add flags
  if (cmd > 0) {
    ifr.ifr_flags = (get_flags() | newflags);
  }
  // del flags
  else if (cmd < 0) {
    ifr.ifr_flags = (get_flags() & ~newflags);
  }
  // set flags
  else {
    ifr.ifr_flags = newflags;
  }

  // open ipv4 control socket
  ACE_SOCK_Dgram ctrlsock (ACE_INET_Addr ("0.0.0.0:0", AF_INET));

  // set flags
  if (ACE_OS::ioctl (ctrlsock.get_handle (), SIOCSIFFLAGS, &ifr) < 0) {
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                            ABORT_LEVEL, 
                            "TunTap::%s:setflags:error %s", 
                            __func__, 
                            ACE_OS::strerror (errno));

    // fail
    return -1;
  }

  // close control socket
  ctrlsock.close();

  // success
  return 0;
}



int EMANE::Transports::Virtual::TunTap::get_flags ()
{
  // interface info
  struct ifreq ifr;

  // clear ifr
  ACE_OS::memset (&ifr, 0, sizeof (ifr));

  // copy dev name
  ACE_OS::strncpy (ifr.ifr_name, tunName_.c_str (), sizeof (ifr.ifr_name));

  // open ipv4 control socket
  ACE_SOCK_Dgram ctrlsock (ACE_INET_Addr ("0.0.0.0:0", AF_INET));

  // get flags
  if (ACE_OS::ioctl (ctrlsock.get_handle (), SIOCGIFFLAGS, &ifr) < 0) {
    LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                            ABORT_LEVEL, 
                            "TunTap::%s:getflags:error %s", 
                            __func__, 
                            ACE_OS::strerror (errno));

    // fail
    return -1;
  }

  // close control socket
  ctrlsock.close();

  // return flags
  return ifr.ifr_flags;
}

#endif
