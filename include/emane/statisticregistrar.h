/*
 * Copyright (c) 2013-2014,2016 - Adjacent Link LLC, Bridgewater, New
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

#ifndef EMANESTATISTICREGISTRAR_HEADER_
#define EMANESTATISTICREGISTRAR_HEADER_

#include "emane/statistic.h"
#include "emane/statisticnumeric.h"
#include "emane/statisticnonnumeric.h"
#include "emane/statistictable.h"
#include "emane/statisticproperties.h"

#include <string>
#include <functional>

namespace EMANE
{
  /**
   * @class StatisticRegistrar
   *
   * @brief The StatisticRegistrar allows NEM layers to register
   * statistics and statistic tables. Statistics and Statistic tables
   * are owned by the emulator framework and a borrowed reference is
   * returned to the registering NEM layer.
   *
   * @note Registration may only occur during Component::initialize()
   *
   */
  class StatisticRegistrar
  {
  public:
    /**
     * Destroys an instance
     */
    virtual ~StatisticRegistrar(){}

    /**
     * Register a numeric statistic. The registered statistic
     * is owned by the StatisticRegistrar.
     *
     * @param sName Name of the statistic
     * @param properties Statistic properties
     * @param sDescription Statistic description
     *
     * @return A borrowed reference to the statistic that may
     * be used during the lifetime of the registered component.
     *
     * @throw RegistrarException when a error occurs during
     * registration.
     */
    template<typename T>
    StatisticNumeric<T> * registerNumeric(const std::string & sName,
                                          const StatisticProperties & properties = StatisticProperties::NONE,
                                          const std::string & sDescription = "");

    /**
     * Register a non-numeric statistic. The registered statistic
     * is owned by the StatisticRegistrar.
     *
     *
     * @param sName Name of the statistic
     * @param properties Statistic properties
     * @param sDescription Statistic description
     *
     * @return A borrowed reference to the statistic that may
     * be used during the lifetime of the registered component.
     *
     * @throw RegistrarException when a error occurs during
     * registration.
     */
    template<typename T>
    StatisticNonNumeric<T> * registerNonNumeric(const std::string & sName,
                                                const StatisticProperties & properties = StatisticProperties::NONE,
                                                const std::string & sDescription = "");


    /**
     * Register a statistic table. The registered statistic table
     * is owned by the StatisticRegistrar.
     *
     * @tparam Key Type of the keys. Each row in the table is uniquely
     *  identified by its key value
     * @tparam Compare A binary predicate that takes two element
     *  keys as arguments and returns a bool
     * @tparam scolumn The column used when sorting the table using
     *  Compare
     *
     * @param sName Name of the statistic table
     * @param labels Table column labels
     * @param properties Table properties
     * @param sDescription Statistic table description
     *
     * @return A borrowed reference to the statistic table that may
     *  be used during the lifetime of the registered component.
     *
     * @throw RegistrarException when a error occurs during
     *  registration.
     */
    template<typename Key,
             typename Compare = std::less<EMANE::Any>,
             std::size_t scolumn = 0>
    StatisticTable<Key,Compare,scolumn> *
    registerTable(const std::string & sName,
                  const StatisticTableLabels & labels,
                  const StatisticProperties & properties = StatisticProperties::NONE,
                  const std::string & sDescription = "");


    /**
     * Register a statistic table. The registered statistic table
     * is owned by the StatisticRegistrar.
     *
     * @tparam Key Type of the keys. Each row in the table is uniquely
     *  identified by its key value
     * @tparam Function A function that takes a pointer to the statistic
     *  table and that is called when the table is cleared
     * @tparam Compare A binary predicate that takes two element
     *  keys as arguments and returns a bool
     * @tparam scolumn The column used when sorting the table using
     *  Compare
     *
     * @param sName Name of the statistic table
     * @param labels Table column labels
     * @param clearFunc Function called when table clear is requested
     * @param sDescription Statistic table description
     *
     * @return A borrowed reference to the statistic table that may
     *  be used during the lifetime of the registered component.
     *
     * @throw RegistrarException when a error occurs during
     *  registration.
     *
     * @note The specified clear function is called on an internal emulator framework thread
     * a synchronization object will most likely me needed to avoid race conditions. The
     * function is responsible for clearing the statistic table.
     */
    template<typename Key,
             typename Function,
             typename Compare = std::less<EMANE::Any>,
             std::size_t scolumn = 0>
    StatisticTable<Key,Compare,scolumn> *
    registerTable(const std::string & sName,
                  const StatisticTableLabels & labels,
                  Function clearFunc,
                  const std::string & sDescription = "");


  protected:
    /**
     * Register a statistic and take ownership
     *
     * @param sName Name of the statistic
     * @param type Any type of the statistic
     * @param properties Statistic propertied
     * @param sDescription Statistic description
     * @param pStatistic Pointer to the statistic
     *
     * @throw RegistrarException when a error occurs during
     * registration.
     */
    virtual void registerStatistic(const std::string & sName,
                                   Any::Type type,
                                   const StatisticProperties & properties,
                                   const std::string & sDescription,
                                   Statistic * pStatistic) = 0;

    /**
     * Register a statistic table publisher and take ownership
     *
     * @param sName Name of the statistic table
     * @param properties Table properties
     * @param sDescription Statistic table description
     * @param pStatiticTablePublisher Pointer to the statistic table
     *  publisher
     * @param clearFunc Function called when table clear is requested
     *
     * @throw RegistrarException when a error occurs during
     * registration.
     */
    virtual void registerTablePublisher(const std::string & sName,
                                        const StatisticProperties & properties,
                                        const std::string & sDescription,
                                        StatisticTablePublisher * pStatiticTablePublisher,
                                        std::function<void(StatisticTablePublisher * p)> clearFunc) = 0;
  };
}

#include "emane/statisticregistrar.inl"

#endif // EMANESTATISTICREGISTRAR_HEADER_

/**
 * @page StatisticService Statistic Service
 *
 * The @ref EMANE::StatisticRegistrar "StatisticRegistrar" is used by components to create
 * statistics and statistic tables. Statistics and statistic tables are owned by the emulator
 * framework so @ref EMANE::StatisticRegistrar::registerNumeric "StatisticRegistrar::registerNumeric",
 * @ref EMANE::StatisticRegistrar::registerNonNumeric "StatisticRegistrar::registerNonNumeric" and
 * @ref EMANE::StatisticRegistrar::registerTable "StatisticRegistrar::registerTable" all return
 * borrowed references. The framework takes care of cleaning these up after all component plugins
 * have been destroyed.
 *
 * @section RegisteringStatistics Registering Statistics
 *
 * Statistics and statistic tables can only be registered during @ref EMANE::Component::initialize
 * "Component::initialize". The @ref  EMANE::StatisticRegistrar "StatisticRegistrar" is accessible via the
 *  @ref EMANE::Component::initialize "initialize" method's @ref EMANE::Registrar "Registrar" argument.
 *
 * There are two types of statistics:
 *
 * - @ref EMANE::StatisticNumeric "StatisticNumeric"
 * - @ref EMANE::StatisticNonNumeric "StatisticNonNumeric"
 *
 * Both statistic types are templates and are thread safe. @ref EMANE::StatisticNumeric "StatisticNumeric"
 * instances can be instantiated with the following types:
 * - std::int64_t
 * - std::int32_t
 * - std::int16_t
 * - std::int8_t
 * - std::uint64_t
 * - std::uint32_t
 * - std::uint16_t
 * - std::uint8_t
 * - bool
 * - float
 * - double
 *
 * @ref EMANE::StatisticNonNumeric "StatisticNonNumeric" template instances can be instantiated with the
 * following types:
 * - std::string
 * - INETAddr
 *
 * Statistics can be given the @ref EMANE::StatisticProperties::CLEARABLE "StatisticProperties::CLEARABLE"
 * property during registration. This property lets client applications clear the statistic using the
 * <a href="https://github.com/adjacentlink/emane-control-port-tutorial">Control Port API</a>.
 *
 * The following snippet shows an example of registering @a clearable statistics.
 *
 * @snippet src/libemane/commonlayerstatistics.cc statisticservice-registernumeric-snippet
 *
 * @section RegisteringStatisticTables Registering Statistic Tables
 *
 * A @ref EMANE::StatisticTable "StatisticTable" is a two dimensional table of @ref EMANE::Any "Any"
 * instances. An @ref EMANE::Any "Any" instance is sort of a @a smart union. It can hold any of the types
 * allowed when creating instances of @ref EMANE::StatisticNumeric "StatisticNumeric" and
 * @ref EMANE::StatisticNonNumeric "StatisticNonNumeric".
 *
 * Statistic Table rules:
 * - Every row in a statistic table must contain the same number of columns.
 * - Each entry in a column may have a different @ref EMANE::Any "Any" type although doing so may make
 *   your table hard to interpret.
 * - You may change the @ref EMANE::Any "Any" type of an entry on every update since you are using
 * @ref EMANE::Any "Any" instances - this would be even harder to interpret.
 *
 * You specify the number of columns in a table by setting the table column names. The number of column
 * names will equal the number of columns in the table. The template parameters to @ref
 * EMANE::StatisticTable "StatisticTable" allow you to set the table key type, the column index that will
 * be used to sort the table and a binary predicate that takes two element keys as arguments and returns a
 * bool which is used to sort the table. The template parameter defaults are usually what you want.
 *
 * @snippet src/libemane/eventtablepublisher.cc statisticservice-registertable-snippet
 *
 * Just like statistics, statistic tables can be @a clearable. Clearing a statistic table is not as
 * straight forward as clearing a statistic. In order to manage statistic table entries you will need
 * to keep track of which items are in the table and what their current values are. An external clear
 * request may be like pulling the rug out from under your table management logic.
 *
 * To address this you need to pass a callable object that takes a single @ref EMANE::StatisticTable
 * "StatisticTable" * as an argument and returns void.  When a table clear request is made the emulator
 * will call the callable to allow it to clear the table. This is one of the few instances where the
 * framework will call a component method from an internal thread. So even though a @ref
 * EMANE::StatisticTable "StatisticTable" is thread safe, execution of this callable is not. If you are
 * accessing shared data in the callable you must use a synchronization object.
 *
 * When you register a table with a callable it automatically applies the
 * @ref EMANE::StatisticProperties::CLEARABLE "CLEARABLE" property to the table. If you wish to register
 * a clearable table without a callable use the other form of the method and manually set the
 * @ref EMANE::StatisticProperties::CLEARABLE "CLEARABLE" property.
 *
 * @snippet src/libemane/commonlayerstatistics.cc statisticservice-registertableclear-snippet
 */
