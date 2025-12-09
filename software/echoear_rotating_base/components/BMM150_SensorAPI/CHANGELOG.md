# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2025-01-10

### Added
- Initial release of BMM150 AUX adapter component
- Support for BMM150 magnetometer through BMI270's AUX interface
- Complete BMM150 API wrapper with AUX interface support
- Utility functions for heading calculation and field strength analysis
- Comprehensive error handling and logging
- Example usage code demonstrating full workflow
- Documentation and API reference

### Features
- AUX interface read/write callback functions
- BMM150 initialization and configuration
- Magnetometer data reading and processing
- Heading calculation from magnetic field data
- Magnetic field strength calculation
- Field normality checking
- Direction string generation
- Register-level access for advanced users

### Dependencies
- ESP-IDF v4.4 or higher
- BMI270 component (espressif2022/bmi270) 