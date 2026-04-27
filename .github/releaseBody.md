**New features:**
- Added Baro altitude and vertical speed support via lookup tables (LUT) with comprehensive tests
- Added packed bitfield encoding for RC channels to reduce memory footprint

**Improvements:**
- Added CRSF wire sizes as constants
- Added compile-time static assertions to validate payload struct sizes at build time
- Extended valid CRSF address range for broader protocol support

**Bugfix:**
- Fixed CMakeLists.txt for compilation on case-sensitive file systems (Linux/macOS)
- Added missing trailing newlines to PPM and PWM source files

See [Changelog](Changelog.md)