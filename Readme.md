<p align="center"> <img src="https://github.com/Tellicious/ADV-utils/assets/9076397/3ec512f1-2de6-4226-bc07-e4bfdd686a28" width=50% height=50%> </p>

# Collection of useful C utilities

[![Unit tests](https://github.com/Tellicious/ADV-utils/actions/workflows/runTests.yml/badge.svg?branch=main&event=push&event=workflow_dispatch)](https://github.com/Tellicious/ADV-utils/actions/workflows/runTests.yml)
[![Codecov](https://codecov.io/gh/Tellicious/ADV-utils/graph/badge.svg?token=OJG3076HXJ)](https://codecov.io/gh/Tellicious/ADV-utils)

## Libraries included:
- ***iBus:*** xxx
- ***SymaX:*** xxx
- ***CRSF:*** xxx

## Library configuration:
- User must define `ADVUTILS_USE_STATIC_ALLOCATION` and/or `ADVUTILS_USE_DYNAMIC_ALLOCATION` to select wheter to use static and/or dynamic memory management. `list`, `LPHashTable` and `LKHashTable` are available only with `ADVUTILS_USE_DYNAMIC_ALLOCATION`
- Functions that use static allocation are defined by `Static` suffix

## Specific configurations (via defines):
  
### list:
- `LIST_STYPE`: type of list size property, defining the maximum amount of values that can be stored in the list. Default is `uint16_t`
  
### LKHashTable:
- `LKHT_LIST_SIZE`: maximum size of the list objects used by the hash-table. Default it `UINT16_MAX`
- `LKHT_HASHFUN`: hash function used by hash-table. Default is `hash_FNV1A` (FNV-1a hash) and other options are `hash_djb` (Dan Bernstein hash) and `hash_sdbm` (sdbm hash)

### LPHashTable:
- `LPHT_MIN_SIZE`: minimum size of hash-table when resizing. Default is `5`
- `LPHT_MAX_SIZE`: maximum size of hash-table when resizing. Default is `UINT32_MAX`
- `LPHT_MIN_SATURATION`: minimum saturation of hash-table, that triggers a down-sizing. Default is `0.2`, meaning `20%`
- `LPHT_MAX_SATURATION`: maximum saturation of hash-table, that triggers an up-sizing. Default is `0.7`, meaning `70%`
- `LPHT_HASHFUN`: hash function used by hash-table. Default is `hash_FNV1A` (FNV-1a hash) and other options are `hash_djb` (Dan Bernstein hash) and `hash_sdbm` (sdbm hash)

### movingAvg:
- `MOVAVG_STYPE`: type of moving average size property, defining the maximum amount of values that can be stored in the moving average. Default is `uint16_t`

### queue:
- `QUEUE_STYPE`: type of queue size property, defining the maximum amount of values that can be stored in the queue. Default is `uint16_t`

## Tests coverage:
| Object      | Coverage |
| ----------- | -------: |
| basicMath   |     100% |
| button      |     100% |
| event       |     100% |
| IIRFilters  |     100% |
| list        |     100% |
| LKHashTable |     100% |
| LPHashTable |     100% |
| matrix      |     100% |
| movingAvg   |     100% |
| numMethods  |     100% |
| PID         |     100% |
| quaternion  |     100% |
| queue       |     100% |
| timer       |     100% |