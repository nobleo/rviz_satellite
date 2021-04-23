/* Copyright 2018-2019 TomTom N.V.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */

#pragma once

#include <unordered_map>

namespace detail
{
/**
 * Manage error rates from entities of type T
 */
template <typename T>
struct ErrorRateManager
{
  struct ErrorRate
  {
    int total_num;
    int error_num;
  };
  std::unordered_map<T, ErrorRate> error_rates;

  /**
   * Calculate the error rate of an entity
   */
  float calculate(T const& id) const
  {
    auto it = error_rates.find(id);
    if (it == error_rates.end())
    {
      return 0;
    }

    auto const& rate = it->second;

    if (rate.total_num == 0)
    {
      return 0;
    }

    return static_cast<float>(rate.error_num) / rate.total_num;
  }

  /**
   * Adjust error rate by counting an error
   */
  void issueError(T const& id)
  {
    auto& rate = error_rates[id];
    ++rate.total_num;
    ++rate.error_num;
  }

  /**
   * Adjust error rate by counting a success
   */
  void issueSuccess(T const& id)
  {
    ++error_rates[id].total_num;
  }
};
}  // namespace detail
