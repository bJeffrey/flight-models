// Test-only replacement operators make warmed-path heap allocation a failure.
// Registration and first-fill snapshot strings are intentionally outside the guard.
#include <cstddef>
#include <cstdlib>
#include <new>

namespace {
thread_local bool allocations_forbidden = false;
}

namespace test_allocation {
void forbid_heap_allocations(bool forbid) noexcept { allocations_forbidden = forbid; }
}

void* operator new(std::size_t size) {
  if (allocations_forbidden) throw std::bad_alloc();
  if (void* memory = std::malloc(size ? size : 1)) return memory;
  throw std::bad_alloc();
}
void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t) noexcept { std::free(memory); }
