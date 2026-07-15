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

#include "FastrtpsCDRWriter.hpp"

#include <bit>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include "fastcdr/Cdr.h"
#include "fastcdr/FastBuffer.h"

#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_c/identifier.h"

#include "rcutils/error_handling.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/message_initialization.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"

namespace rmw_cyclonedds_cpp
{

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static bool host_needs_bswap_fr()
{
  return std::endian::native == std::endian::big;
}

// The 4-byte RTPS/CDR encoding header written by cyclone before the payload.
// Byte 0: 0x00 (CDR)
// Byte 1: 0x01 = LE, 0x00 = BE
// Bytes 2-3: options = 0x00 0x00
static void write_encoding_header(void * dest)
{
  unsigned char hdr[4] = {0x00, static_cast<unsigned char>(host_needs_bswap_fr() ? 0x00 : 0x01),
    0x00, 0x00};
  std::memcpy(dest, hdr, 4);
}

// ---------------------------------------------------------------------------
// FastrtpsCDRWriter
// ---------------------------------------------------------------------------

FastrtpsCDRWriter::FastrtpsCDRWriter(
  const message_type_support_callbacks_t * callbacks,
  SampleOrRequest variant)
: m_callbacks(callbacks), m_variant(variant)
{
  char bounds_info = 0;
  size_t max = callbacks->max_serialized_size(bounds_info);
  if (bounds_info == ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE) {
    m_max_size = std::numeric_limits<size_t>::max();
  } else {
    m_max_size = 4 + max;  // include encoding header
  }
  m_has_keys = (callbacks->key_callbacks != nullptr);
}

size_t FastrtpsCDRWriter::get_serialized_size(const void * data, SampleOrKey what) const
{
  if (what == SampleOrKey::Key) {
    if (!m_has_keys) {
      return 4;
    }
    return 4 + m_callbacks->key_callbacks->get_serialized_size_key(data);
  }
  return 4 + static_cast<size_t>(m_callbacks->get_serialized_size(data));
}

size_t FastrtpsCDRWriter::get_serialized_size_estimate(const void * data, SampleOrKey what) const
{
  return get_serialized_size(data, what);
}

size_t FastrtpsCDRWriter::get_min_serialized_size(SampleOrKey /*what*/) const
{
  return 4;
}

size_t FastrtpsCDRWriter::get_max_serialized_size(SampleOrKey what) const
{
  if (what == SampleOrKey::Key) {
    if (!m_has_keys) {
      return 4;
    }
    bool is_unbounded = false;
    size_t max_key = m_callbacks->key_callbacks->max_serialized_size_key(is_unbounded);
    if (is_unbounded) {
      return std::numeric_limits<size_t>::max();
    }
    return 4 + max_key;
  }
  return m_max_size;
}

void FastrtpsCDRWriter::serialize(void * dest, const void * data, SampleOrKey what) const
{
  // Write 4-byte encoding header
  write_encoding_header(dest);

  unsigned char * payload = static_cast<unsigned char *>(dest) + 4;

  // Compute payload capacity conservatively: use get_serialized_size.
  // The buffer was pre-allocated by cyclone based on our get_serialized_size() above,
  // so we know it's large enough — just wrap it with a large FastBuffer.
  // We use a sentinel size that covers any bounded message; for unbounded we
  // allocate on the fly. Use 64 MiB sentinel cap; FastBuffer won't allocate.
  constexpr size_t kSentinelCap = 64u * 1024u * 1024u;

  if (what == SampleOrKey::Key) {
    if (!m_has_keys) {
      return;
    }
    eprosima::fastcdr::FastBuffer buf(reinterpret_cast<char *>(payload), kSentinelCap);
    eprosima::fastcdr::Cdr cdr(
      buf,
      host_needs_bswap_fr()
      ? eprosima::fastcdr::Cdr::BIG_ENDIANNESS
      : eprosima::fastcdr::Cdr::LITTLE_ENDIANNESS,
      eprosima::fastcdr::CdrVersion::XCDRv1);
    if (!m_callbacks->key_callbacks->cdr_serialize_key(data, cdr)) {
      throw std::runtime_error("FastrtpsCDRWriter: key serialization failed");
    }
    return;
  }

  eprosima::fastcdr::FastBuffer buf(reinterpret_cast<char *>(payload), kSentinelCap);
  eprosima::fastcdr::Cdr cdr(
    buf,
    host_needs_bswap_fr()
    ? eprosima::fastcdr::Cdr::BIG_ENDIANNESS
    : eprosima::fastcdr::Cdr::LITTLE_ENDIANNESS,
    eprosima::fastcdr::CdrVersion::XCDRv1);

  if (!m_callbacks->cdr_serialize(data, cdr)) {
    throw std::runtime_error("FastrtpsCDRWriter: serialization failed");
  }
}

TypeGenerator FastrtpsCDRWriter::type_generator() const
{
  return TypeGenerator::ROSIDL_Cpp;
}

// ---------------------------------------------------------------------------
// FastrtpsCDRReader
// ---------------------------------------------------------------------------

FastrtpsCDRReader::FastrtpsCDRReader(
  const message_type_support_callbacks_t * callbacks,
  MessageMembersVariant introspection,
  SampleOrRequest variant)
: m_callbacks(callbacks), m_introspection(introspection), m_variant(variant),
  m_tmp_sample(
    std::visit([](auto * m) { return m->size_of_; }, introspection), std::byte{0})
{
  std::visit(
    [this](auto * m) {
      using M = std::remove_const_t<std::remove_pointer_t<decltype(m)>>;
      if constexpr (std::is_same_v<M, rosidl_typesupport_introspection_cpp::MessageMembers>) {
        m->init_function(
          m_tmp_sample.data(), rosidl_runtime_cpp::MessageInitialization::ALL);
      } else {
        m->init_function(
          m_tmp_sample.data(), ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      }
    }, m_introspection);
}

FastrtpsCDRReader::~FastrtpsCDRReader()
{
  std::visit(
    [this](auto * m) { m->fini_function(m_tmp_sample.data()); },
    m_introspection);
}

void FastrtpsCDRReader::deserialize(
  void * dst, const void * cdr_raw, size_t cdrsize, SampleOrKey what) const
{
  if (cdrsize < 4) {
    throw std::runtime_error("FastrtpsCDRReader: CDR buffer too small");
  }

  const unsigned char * hdr = static_cast<const unsigned char *>(cdr_raw);
  if (hdr[0] != 0 || hdr[1] > 1) {
    throw std::runtime_error("FastrtpsCDRReader: unrecognized CDR encoding header");
  }
  const bool src_is_le = (hdr[1] == 1);

  const char * payload = static_cast<const char *>(cdr_raw) + 4;
  const size_t payload_size = cdrsize - 4;

  eprosima::fastcdr::FastBuffer buf(const_cast<char *>(payload), payload_size);
  eprosima::fastcdr::Cdr cdr(
    buf,
    src_is_le
    ? eprosima::fastcdr::Cdr::LITTLE_ENDIANNESS
    : eprosima::fastcdr::Cdr::BIG_ENDIANNESS,
    eprosima::fastcdr::CdrVersion::XCDRv1);

  if (what == SampleOrKey::Key) {
    if (m_callbacks->key_callbacks) {
      // No cdr_deserialize_key in the public API; fall through to sample deserialization
      // for key-only reads — cyclone only uses extractkey() for key extraction, not deserialize().
    }
    return;
  }

  if (m_variant == SampleOrRequest::Request) {
    // Service request: 16-byte request header prepended before payload.
    // The header is already consumed by cyclone before calling us; dst points
    // to the cdds_request_wrapper_t. We skip the 16-byte guid+seq here.
    // (Same approach as CDRDeserializer: advance 16 bytes, then deserialize into ->data)
    struct RequestHeader { uint64_t guid; uint64_t seq; };
    RequestHeader rh;
    cdr.deserialize_array(reinterpret_cast<uint8_t *>(&rh), sizeof(rh));
    struct RequestWrapper { RequestHeader header; void * data; };
    auto * req = static_cast<RequestWrapper *>(dst);
    req->header = rh;
    if (!m_callbacks->cdr_deserialize(cdr, req->data)) {
      throw std::runtime_error("FastrtpsCDRReader: deserialization of request payload failed");
    }
    return;
  }

  if (!m_callbacks->cdr_deserialize(cdr, dst)) {
    throw std::runtime_error("FastrtpsCDRReader: deserialization failed");
  }
}

void FastrtpsCDRReader::extractkey(
  std::vector<std::byte> & dst, const void * cdr_raw, size_t cdrsize,
  SampleOrKey /*what*/) const
{
  if (!m_callbacks->key_callbacks) {
    return;
  }

  if (cdrsize < 4) {
    throw std::runtime_error("FastrtpsCDRReader::extractkey: CDR buffer too small");
  }

  const unsigned char * hdr = static_cast<const unsigned char *>(cdr_raw);
  const bool src_is_le = (hdr[1] == 1);
  const char * payload = static_cast<const char *>(cdr_raw) + 4;
  const size_t payload_size = cdrsize - 4;

  // Step 1: deserialize the full sample into the preallocated tmp buffer.
  eprosima::fastcdr::FastBuffer rbuf(const_cast<char *>(payload), payload_size);
  eprosima::fastcdr::Cdr rcdr(
    rbuf,
    src_is_le
    ? eprosima::fastcdr::Cdr::LITTLE_ENDIANNESS
    : eprosima::fastcdr::Cdr::BIG_ENDIANNESS,
    eprosima::fastcdr::CdrVersion::XCDRv1);

  void * tmp = m_tmp_sample.data();
  std::visit(
    [tmp](auto * m) {
      using M = std::remove_const_t<std::remove_pointer_t<decltype(m)>>;
      if constexpr (std::is_same_v<M, rosidl_typesupport_introspection_cpp::MessageMembers>) {
        m->init_function(tmp, rosidl_runtime_cpp::MessageInitialization::ALL);
      } else {
        m->init_function(tmp, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      }
    }, m_introspection);
  if (!m_callbacks->cdr_deserialize(rcdr, tmp)) {
    throw std::runtime_error("FastrtpsCDRReader::extractkey: deserialization failed");
  }

  // Step 2: re-serialize only the key fields into dst using key callbacks.
  bool is_unbounded = false;
  const size_t max_key = m_callbacks->key_callbacks->max_serialized_size_key(is_unbounded);
  const size_t key_buf_size = is_unbounded
    ? 64u * 1024u * 1024u
    : max_key + 4;
  dst.resize(key_buf_size);

  eprosima::fastcdr::FastBuffer wbuf(reinterpret_cast<char *>(dst.data()), key_buf_size);
  eprosima::fastcdr::Cdr wcdr(
    wbuf,
    host_needs_bswap_fr()
    ? eprosima::fastcdr::Cdr::BIG_ENDIANNESS
    : eprosima::fastcdr::Cdr::LITTLE_ENDIANNESS,
    eprosima::fastcdr::CdrVersion::XCDRv1);

  if (!m_callbacks->key_callbacks->cdr_serialize_key(tmp, wcdr)) {
    throw std::runtime_error("FastrtpsCDRReader::extractkey: key serialization failed");
  }

  dst.resize(wcdr.get_serialized_data_length());

  std::visit([tmp](auto * m) { m->fini_function(tmp); }, m_introspection);
}

void FastrtpsCDRReader::extractkey_be(
  std::vector<std::byte> & dst, const void * cdr_raw, size_t cdrsize,
  SampleOrKey what) const
{
  extractkey(dst, cdr_raw, cdrsize, what);
}

size_t FastrtpsCDRReader::print(
  char * dst, size_t dstsize, const void * /*cdr*/, size_t /*cdrsize*/,
  SampleOrKey /*what*/) const
{
  return static_cast<size_t>(snprintf(dst, dstsize, "<fastrtps-backed message>"));
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

template<typename MessageMembers>
static bool has_wstring_impl(const MessageMembers * members)
{
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto & m = members->members_[i];
    if (m.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING) {
      return true;
    }
    if (m.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
      const auto * sub = static_cast<const MessageMembers *>(m.members_->data);
      if (has_wstring_impl(sub)) {
        return true;
      }
    }
  }
  return false;
}

static bool has_wstring(const MessageMembersVariant & v)
{
  return std::visit([](auto * m) { return has_wstring_impl(m); }, v);
}

// ---------------------------------------------------------------------------
// Factories
// ---------------------------------------------------------------------------

std::unique_ptr<BaseCDRWriter> make_cdr_writer_fastrtps(
  const rosidl_message_type_support_t * mts,
  SampleOrRequest variant)
{
  // Prefer C++ callbacks; fall back to C callbacks — wire format is identical.
  const rosidl_message_type_support_t * ts =
    get_message_typesupport_handle(mts, rosidl_typesupport_fastrtps_cpp::typesupport_identifier);
  if (!ts) {
    rcutils_reset_error();
    ts = get_message_typesupport_handle(
      mts, rosidl_typesupport_fastrtps_c__identifier);
  }
  if (!ts) {
    rcutils_reset_error();
    return nullptr;
  }
  const auto * callbacks =
    static_cast<const message_type_support_callbacks_t *>(ts->data);
  return std::make_unique<FastrtpsCDRWriter>(callbacks, variant);
}

std::unique_ptr<BaseCDRReader> make_cdr_reader_fastrtps(
  const rosidl_message_type_support_t * mts,
  SampleOrRequest variant)
{
  // Try C++ fastrtps + C++ introspection first.
  const rosidl_message_type_support_t * ts_fr =
    get_message_typesupport_handle(mts, rosidl_typesupport_fastrtps_cpp::typesupport_identifier);
  const rosidl_message_type_support_t * ts_intr =
    ts_fr
    ? get_message_typesupport_handle(mts, rosidl_typesupport_introspection_cpp::typesupport_identifier)
    : nullptr;
  if (ts_fr && ts_intr) {
    const auto * callbacks =
      static_cast<const message_type_support_callbacks_t *>(ts_fr->data);
    const auto * intr =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts_intr->data);
    MessageMembersVariant v{intr};
    if (!has_wstring(v)) {
      return std::make_unique<FastrtpsCDRReader>(callbacks, v, variant);
    }
    rcutils_reset_error();
    return nullptr;  // wstring types unsupported via fastrtps (different wire format from cyclone)
  }
  if (ts_fr) {
    rcutils_reset_error();
  }

  // Fall back to C fastrtps + C introspection (rclpy / C endpoints).
  ts_fr = get_message_typesupport_handle(mts, rosidl_typesupport_fastrtps_c__identifier);
  if (!ts_fr) {
    rcutils_reset_error();
    return nullptr;
  }
  ts_intr = get_message_typesupport_handle(
    mts, rosidl_typesupport_introspection_c__identifier);
  if (!ts_intr) {
    rcutils_reset_error();
    return nullptr;
  }
  const auto * callbacks =
    static_cast<const message_type_support_callbacks_t *>(ts_fr->data);
  const auto * intr =
    static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(ts_intr->data);
  MessageMembersVariant v{intr};
  if (has_wstring(v)) {
    return nullptr;
  }
  return std::make_unique<FastrtpsCDRReader>(callbacks, v, variant);
}

}  // namespace rmw_cyclonedds_cpp
