// Copyright 2024 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef FASTRTPS_CDR_WRITER_HPP_
#define FASTRTPS_CDR_WRITER_HPP_

#include <cstddef>
#include <memory>

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

#include "BaseCDRWriter.hpp"
#include "BaseCDRReader.hpp"
#include "TypeSupport2.hpp"

namespace rmw_cyclonedds_cpp
{

// ---------------------------------------------------------------------------
// FastrtpsCDRWriter — delegates serialize/size to rosidl_typesupport_fastrtps_cpp callbacks.
// ---------------------------------------------------------------------------
class FastrtpsCDRWriter final : public BaseCDRWriter
{
public:
  explicit FastrtpsCDRWriter(
    const message_type_support_callbacks_t * callbacks,
    SampleOrRequest variant);

  size_t get_serialized_size(const void * data, SampleOrKey what) const override;
  size_t get_serialized_size_estimate(const void * data, SampleOrKey what) const override;
  size_t get_min_serialized_size(SampleOrKey what) const override;
  size_t get_max_serialized_size(SampleOrKey what) const override;
  void serialize(void * dest, const void * data, SampleOrKey what) const override;
  TypeGenerator type_generator() const override;

private:
  const message_type_support_callbacks_t * m_callbacks;
  SampleOrRequest m_variant;
  size_t m_max_size;        // SIZE_MAX if unbounded
  bool m_has_keys;
};

// ---------------------------------------------------------------------------
// FastrtpsCDRReader — delegates deserialize to rosidl_typesupport_fastrtps_cpp callbacks.
// ---------------------------------------------------------------------------
class FastrtpsCDRReader final : public BaseCDRReader
{
public:
  explicit FastrtpsCDRReader(
    const message_type_support_callbacks_t * callbacks,
    MessageMembersVariant introspection,
    SampleOrRequest variant);
  ~FastrtpsCDRReader();

  void deserialize(
    void * dst, const void * cdr, size_t cdrsize,
    SampleOrKey what) const override;

  void extractkey(
    std::vector<std::byte> & dst, const void * cdr, size_t cdrsize,
    SampleOrKey what) const override;

  void extractkey_be(
    std::vector<std::byte> & dst, const void * cdr, size_t cdrsize,
    SampleOrKey what) const override;

  size_t print(
    char * dst, size_t dstsize, const void * cdr, size_t cdrsize,
    SampleOrKey what) const override;

private:
  const message_type_support_callbacks_t * m_callbacks;
  MessageMembersVariant m_introspection;
  SampleOrRequest m_variant;
  mutable std::vector<std::byte> m_tmp_sample;  // preallocated native sample buffer
};

// ---------------------------------------------------------------------------
// Try to build a fastrtps-backed writer/reader from a raw type support handle.
// Returns nullptr if the fastrtps typesupport handle is not available.
// ---------------------------------------------------------------------------
std::unique_ptr<BaseCDRWriter> make_cdr_writer_fastrtps(
  const rosidl_message_type_support_t * mts,
  SampleOrRequest variant);

std::unique_ptr<BaseCDRReader> make_cdr_reader_fastrtps(
  const rosidl_message_type_support_t * mts,
  SampleOrRequest variant);

}  // namespace rmw_cyclonedds_cpp
#endif  // FASTRTPS_CDR_WRITER_HPP_
