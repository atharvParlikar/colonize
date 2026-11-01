// Headless checks of engine pieces that need no window.
// Build target: engine-tests. Exit code is the number of failed checks.

#include "engine/collision/queries.hpp"
#include "engine/entity/pool.hpp"

#include <cmath>

#include <cstdio>
#include <vector>

namespace {

int failures = 0;

void check(const char *name, bool ok) {
  failures += ok ? 0 : 1;
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
}

struct Thing {
  int value = 0;
};

using engine::Handle;
using engine::Pool;

void poolBasics() {
  std::printf("pool: create, get, destroy\n");
  Pool<Thing> pool;
  check("starts empty", pool.empty() && pool.size() == 0);
  check("default handle is null", !Handle<Thing>{} && !pool.get(Handle<Thing>{}));

  const Handle<Thing> a = pool.create(Thing{1});
  const Handle<Thing> b = pool.create(Thing{2});
  check("handles are truthy", static_cast<bool>(a) && static_cast<bool>(b));
  check("get finds each", pool.get(a) && pool.get(a)->value == 1 && pool.get(b) && pool.get(b)->value == 2);
  check("size counts live", pool.size() == 2);

  check("destroy succeeds once", pool.destroy(a));
  check("destroy of a stale handle fails", !pool.destroy(a));
  check("destroyed handle is stale", pool.get(a) == nullptr && !pool.contains(a));
  check("the other survives", pool.get(b) && pool.get(b)->value == 2);
  check("size drops", pool.size() == 1);
}

void poolReuse() {
  std::printf("pool: slot reuse keeps old handles stale\n");
  Pool<Thing> pool;
  const Handle<Thing> old = pool.create(Thing{1});
  pool.destroy(old);
  const Handle<Thing> fresh = pool.create(Thing{2});
  check("slot is reused", fresh.index == old.index);
  check("generation differs", fresh.generation != old.generation);
  check("old handle stays stale", pool.get(old) == nullptr);
  check("new handle sees the new entity", pool.get(fresh) && pool.get(fresh)->value == 2);
  check("recreated entity starts from its own value", pool.get(fresh)->value == 2);
}

void poolIteration() {
  std::printf("pool: iteration\n");
  Pool<Thing> pool;
  std::vector<Handle<Thing>> handles;
  for (int i = 0; i < 6; ++i) handles.push_back(pool.create(Thing{i}));
  pool.destroy(handles[1]);
  pool.destroy(handles[4]);

  int sum = 0, count = 0;
  bool handlesMatch = true;
  for (auto [handle, thing] : pool) {
    sum += thing.value;
    ++count;
    handlesMatch = handlesMatch && pool.get(handle) == &thing;
  }
  check("visits only live entities", count == 4 && sum == 0 + 2 + 3 + 5);
  check("yielded handles point at the yielded entity", handlesMatch);

  // Destroy every other one while iterating.
  count = 0;
  for (auto [handle, thing] : pool) {
    if (thing.value % 2 == 0) pool.destroy(handle);
    ++count;
  }
  check("destroying mid-loop still visits each once", count == 4);
  check("and leaves the rest", pool.size() == 2);

  // Create while iterating (may reallocate).
  Pool<Thing> growing;
  growing.create(Thing{100});
  int visited = 0;
  for (auto [handle, thing] : growing) {
    (void)handle;
    (void)thing;
    ++visited;
    for (int i = 0; i < 64; ++i) growing.create(Thing{i});
  }
  check("creating mid-loop is safe", growing.size() == 65 && visited >= 1);

  const Pool<Thing> &constPool = pool;
  count = 0;
  for (auto [handle, thing] : constPool) {
    (void)handle;
    count += thing.value > 0 ? 1 : 0;
  }
  check("const iteration", count == 2);

  pool.clear();
  check("clear empties", pool.empty());
  check("clear makes handles stale", pool.get(handles[3]) == nullptr);
}

bool near(std::optional<float> t, float expected) { return t && std::fabs(*t - expected) < 1e-4f; }

void sweepQueries() {
  std::printf("collision: segment sweeps\n");
  using engine::segmentAabb;
  using engine::segmentSphere;
  const Vector3 c{0.0f, 0.0f, 0.0f};
  check("sphere: head-on hit at the surface", near(segmentSphere({-10, 0, 0}, {10, 0, 0}, c, 2.0f), 8.0f / 20.0f));
  check("sphere: passes beside", !segmentSphere({-10, 3, 0}, {10, 3, 0}, c, 2.0f));
  check("sphere: stops short", !segmentSphere({-10, 0, 0}, {-5, 0, 0}, c, 2.0f));
  check("sphere: moving away", !segmentSphere({3, 0, 0}, {10, 0, 0}, c, 2.0f));
  check("sphere: starts inside is t = 0", near(segmentSphere({0.5f, 0, 0}, {10, 0, 0}, c, 2.0f), 0.0f));
  check("sphere: grazing hit", segmentSphere({-10, 1.999f, 0}, {10, 1.999f, 0}, c, 2.0f).has_value());

  const BoundingBox box{{-1, -1, -1}, {1, 1, 1}};
  check("box: head-on hit at the face", near(segmentAabb({-5, 0, 0}, {5, 0, 0}, box), 0.4f));
  check("box: diagonal hit", segmentAabb({-5, -5, -5}, {5, 5, 5}, box).has_value());
  check("box: passes beside", !segmentAabb({-5, 2, 0}, {5, 2, 0}, box));
  check("box: stops short", !segmentAabb({-5, 0, 0}, {-2, 0, 0}, box));
  check("box: parallel inside the slab", near(segmentAabb({-5, 0.5f, 0.5f}, {5, 0.5f, 0.5f}, box), 0.4f));
  check("box: starts inside is t = 0", near(segmentAabb({0, 0, 0}, {5, 0, 0}, box), 0.0f));
}

} // namespace

int main() {
  sweepQueries();
  poolBasics();
  poolReuse();
  poolIteration();
  std::printf("\n%d check(s) failed\n", failures);
  return failures;
}
