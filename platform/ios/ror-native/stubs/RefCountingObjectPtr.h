#pragma once

#include <cstddef>

// Parser-only iOS host shim.
//
// RoR's real RefCountingObjectPtr<T> header embeds AngelScript GC/registration
// support. RigDef only needs the pointer type to lay out document fields (for
// example CacheEntryPtr); it never invokes AngelScript or owns those objects in
// this target. Keep the same one-pointer storage shape and C++ pointer surface
// while deliberately omitting AngelScript registration/GC machinery.
template <class T>
class RefCountingObjectPtr
{
public:
    RefCountingObjectPtr() noexcept : m_ref(nullptr) {}
    RefCountingObjectPtr(std::nullptr_t) noexcept : m_ref(nullptr) {}
    explicit RefCountingObjectPtr(T* ref) noexcept : m_ref(ref) {}
    RefCountingObjectPtr(const RefCountingObjectPtr&) noexcept = default;
    RefCountingObjectPtr& operator=(const RefCountingObjectPtr&) noexcept = default;

    T* GetRef() noexcept { return m_ref; }
    T* GetRef() const noexcept { return m_ref; }
    T* operator->() noexcept { return m_ref; }
    T* operator->() const noexcept { return m_ref; }

    bool operator==(const RefCountingObjectPtr& other) const noexcept { return m_ref == other.m_ref; }
    bool operator!=(const RefCountingObjectPtr& other) const noexcept { return m_ref != other.m_ref; }
    bool operator==(const T* other) const noexcept { return m_ref == other; }
    bool operator!=(const T* other) const noexcept { return m_ref != other; }
    bool operator==(std::nullptr_t) const noexcept { return m_ref == nullptr; }
    bool operator!=(std::nullptr_t) const noexcept { return m_ref != nullptr; }

    explicit operator bool() const noexcept { return m_ref != nullptr; }
    operator long long() const noexcept { return reinterpret_cast<long long>(m_ref); }

private:
    T* m_ref;
};
