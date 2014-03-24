/*
 * Copyright (c) 2013-2014 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMANESTATISTICTABLE_HEADER_
#define EMANESTATISTICTABLE_HEADER_

#include "emane/statistictablepublisher.h"

#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <functional>

namespace EMANE
{
  class StatisticRegistrar;

  /**
   * @class StatisticTable
   *
   * @brief A two dimentional statistic table that holds Any values.
   *
   * @tparam Key Type of the keys. Each row in the table is uniquely
   *  identified by its key value
   * @tparam Compare A binary predicate that takes two element
   *  keys as arguments and returns a bool
   * @tparam scolumn The column used when sorting the table using
   *  Compare
   */
  template<typename Key,
           typename Compare = std::less<EMANE::Any>,
           std::size_t sortIndex = 0>
  class StatisticTable : public StatisticTablePublisher
  {
  public:
    /**
     * Destroys an instance
     */
    ~StatisticTable();

    /**
     * Sets a specified table cell
     *
     * @param key Table row key
     * @param columnIndex Row column index (indexed from 0)
     * @param any Value to set
     *
     * @throw StatisticTableException when either the row key of
     * column index are invalid
     */
    void setCell(const Key & key,
                 std::size_t columnIndex,
                 const Any & any);
    

    /**
     * Sets a column of a specified row
     *
     * @param key Table row key
     * @param anys Vector of any values
     *
     * @throw StatisticTableException when either the row key is invalid or
     * the number of columns in the row values does not match the table
     */
    void setRow(const Key & key,
                const std::vector<Any> & anys);

    /**
     * Adds a row to the table
     *
     * @param key Table row key
     * @param anys Vector of any values
     *
     * @throw StatisticTableException when either the row key is already in use or
     * the number of columns in the row values does not match the table
     */
    void addRow(const Key & key,
                const std::vector<Any> & anys = {});

    /**
     * Deletes a row from the table
     *
     * @param key Table row key
     */
    void deleteRow(const Key & key);

    StatisticTableLabels getLabels() const override;
    

    StatisticTableValues getValues() const override;

    void clear() override;

  private:
    using InternalTable = std::unordered_map<Key,std::vector<Any>>;
    InternalTable table_;
    mutable std::mutex mutex_;
    const StatisticTableLabels labels_;
    std::function<void()> clearFunc_;

    StatisticTable(const StatisticTableLabels & labels);
    
    template <typename Function>
    StatisticTable(const StatisticTableLabels & labels,
                   Function clearFunc);

    friend StatisticRegistrar;
  };
}

#include "emane/statistictable.inl"

#endif // EMANESTATISTICTABLE_HEADER_
