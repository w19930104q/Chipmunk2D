# Spatial hash acceptance tests

Tests for the `cpSpaceHash` infinite-loop fix (`src/cpSpaceHash.c`).
Build the library first, e.g.:

```sh
mkdir -p /tmp/chip_build && cd /tmp/chip_build
for f in <repo>/src/*.c; do gcc -O2 -std=gnu99 -I<repo>/include -I<repo>/src -c "$f"; done
ar rcs libchipmunk.a *.o
```

Then build the tests (replace `<repo>` with the repo path):

```sh
gcc -O2 -std=gnu99 -I<repo>/include diff_test.c  /tmp/chip_build/libchipmunk.a -lm -o diff_test
gcc -O2 -std=gnu99 -I<repo>/include -I<repo>/src fuzz.c /tmp/chip_build/libchipmunk.a -lm -o fuzz
gcc -O2 -std=gnu99 -I<repo>/include -I<repo>/src hang_repro.c /tmp/chip_build/libchipmunk.a -lm -o hang_repro
gcc -O2 -std=gnu99 -I<repo>/include -I<repo>/src resize_test.c /tmp/chip_build/libchipmunk.a -lm -o resize_test
```

## diff_test.c — deterministic differential acceptance test

Pre-generates a random op sequence (add / remove / teleport / jitter /
activate / reindex, including moves across hash cell grid lines) and applies
it identically to a spatial-hash space (`cpSpaceUseSpatialHash(space, 5, 500)`)
and a default BB-tree space. Shapes are sensors with zero gravity so the
layouts cannot diverge. After every `cpSpaceStep` it compares sorted shape-id
lists from 8 `cpSpacePointQuery` and 2 `cpSpaceShapeQuery` calls between the
two spaces; any difference exits with status 2.

```sh
./diff_test 2000 1   # 2000 steps, RNG seed 1
```

## fuzz.c — mutation-path fuzzer

14 op types (add/remove/teleport/jitter/reindexShape/reindexShapesForBody/
reindexStatic/activate) on dynamic + kinematic bodies with gravity, sleeping
enabled, and point/segment/BB/shape queries every step. `NOHASH=1` runs the
same sequence against the BB tree as a control.

```sh
./fuzz 2500 1        # 2500 steps, RNG seed 1
```

## hang_repro.c — degenerate-bb hang repro

Bodies teleported to ~1e10 coordinates, NaN/Inf positions (poked via the
private header to bypass debug asserts, as happens in release builds after
physics explosions), a gigantic 1e8-sized shape, and a huge-bb point query.
Build the library with `-DNDEBUG` for the NaN/Inf cases. The pre-fix library
never returns (kill with `timeout`); the fixed library finishes instantly.

## resize_test.c — cpSpaceHashResize consistency

After `cpSpaceHashResize`, static shapes must remain queryable (the old code
emptied the table without rehashing, so static shapes silently disappeared
from query results until a manual rehash).
