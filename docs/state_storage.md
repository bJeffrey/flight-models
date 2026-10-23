# Bounded Physical-State Storage

`flight_models::StateStore` implements FM-SOA-1's initial physical-state storage.
Its API is [state_store.h](../include/flight_models/state_store.h). This is a
transport-independent library: no dynamics, UCI/DIS codec, clock adapter or
numerical backend dispatcher is implemented here.

## Layout and resources

The store allocates numerical columns once for its explicit entity capacity.
Position, velocity, acceleration and body angular velocity use separate
contiguous component arrays. Attitude uses packed row-major 3×3 matrices (nine
values per entity). Source nanoseconds and desired heading/speed/HAE have their
own arrays; occupancy, model-profile index and fixed/moving/disabled mode are
separate metadata. Live views address slots `[0, capacity)` and callers must
check occupancy before using a slot; compact batch views address rows
`[0, batch.size())`. The public API reuses shared `math::Vector3`/`Matrix3` types
and exposes no Eigen/MKL-specific type. Shared math builds inside the maintained
container from the explicitly mounted sibling repository.

[`storage_policy.json`](../config/storage_policy.json) supplies example execution
resource ceilings and an ordered motion-profile registry, separately from
vehicle motion limits. Its schema is checked by the storage test, and model
files use the production profile loader. A future runtime adapter will load this
policy; the storage library itself requires explicit constructor bounds and does
not parse deployment files. The 10,000-entity ceiling is a configurable example,
not measured capacity or a small/large dispatch threshold.

`StateBatch` preallocates numerical staging and handle/profile/mode metadata.
Its capacity is independent of the store's; exceeding it fails explicitly rather
than resizing or evicting entities. Variable-size routes/emitter events and
nonideal navigation state remain with their later domain modules.

## Identity and lifecycle

An external platform ID maps to a slot plus generation. Lowest-free-slot reuse is
deterministic regardless of erase order; a reserved min-heap avoids repeated
full-store scans while populating large scenarios. ID lookup is ordered-map
lookup, and profile/snapshot iteration follows canonical ID order. Erase, reuse
and fresh-run clear invalidate previous handles. Handles and prepared batches
belong to the lifetime of their originating store, not a wire identity.

Fixed and disabled entities remain present in snapshots. A moving-only cohort
excludes them; removal clears occupancy. Profile indices address the explicitly
loaded registry and must not become external model or platform identities.

## Updating small sets and full cohorts

For small or sparse due-update sets, `gather(handles, count, batch)` preserves
caller order and validates only the supplied handles. For a complete compatible
cohort, `gather_profile(profile, mode, batch)` packs matching entities into
contiguous scratch in canonical ID order. Kernels will operate on the same
backend-independent columns in either case. No automatic cutoff is inferred from
allocated capacity, and no Eigen/MKL integration or crossover measurement is
claimed by this slice.

Writes to batch scratch do not change committed storage. Commit checks ownership,
store revision, handles, duplicate rows, finite numerical values and nondecreasing
source time before scattering any row. Failure leaves committed values/revision
unchanged. A successful commit consumes the prepared batch and advances the
revision. Any intervening lifecycle change or other commit requires regathering
before committing another prepared batch. Storage validates numerical shape and
time; model bounds and proper attitude geometry remain model/adapter contracts.

The store is single-owner and calls must be serialized. Workers may operate on
separate rows of a staged batch under the caller's synchronization; join them
before commit. Live views borrow storage and must not be read concurrently with
mutation. This is not a lock-free publication mechanism or a new epoch protocol.

## Snapshots and allocation behavior

`Snapshot` owns preallocated copied rows, per-entity source times and captured
store revision. It remains unchanged when the store is subsequently updated or
slots are reused. Untouched entities retain their old source times; a partial
update does not label them as newly propagated.

Snapshot IDs are owning strings. Initial fill and changed lifecycle metadata can
allocate string storage; numerical gather/commit and stable-population snapshots
reuse their buffers after warmup. The test forbids heap allocation on that warmed
path, including long IDs. This is an allocation invariant, not a throughput or
backend crossover benchmark.

Give asynchronous publishers an owning snapshot and do not refill it while they
read it. Snapshot row pointers remain valid only within that snapshot object's
lifetime and until it is refilled/moved. Coherent interval admission and the point
at which a runtime publishes a completed interval remain runtime responsibilities.
