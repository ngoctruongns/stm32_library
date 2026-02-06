# Motor Control Library - Folder Structure

```
motor_control/
├── inc/                              # Public header files
│   ├── encoder.h                    # Encoder class interface
│   ├── PID_ctl.h                    # PID controller class interface
│   ├── motor_driver.h               # Motor driver (L298) interface
│   ├── motor_ctl.h                  # Main motor controller interface
│   └── motor_control.h              # Master header (includes all)
│
├── src/                              # Implementation files
│   ├── encoder.cpp                  # Encoder implementation
│   ├── PID_ctl.cpp                  # PID controller implementation
│   ├── motor_driver.cpp             # Motor driver implementation
│   └── motor_ctl.cpp                # Main motor controller implementation
│
├── config/                           # Configuration files
│   └── motor_config.h               # Motor parameters and hardware config
│
├── examples/                         # Usage examples
│   └── motor_example.cpp            # Complete example with usage patterns
│
├── docs/                             # Documentation
│   ├── README.md                    # Getting started guide
│   ├── PID_TUNING_GUIDE.md          # Detailed PID tuning instructions
│   └── IMPROVEMENTS_SUGGESTIONS.md  # Future features and enhancements
│
└── STRUCTURE.md                      # This file
```

## How to Include in Your Projects

### Include all classes:
```c
#include "Library/motor_control/inc/motor_control.h"
```

### Or include specific classes:
```c
#include "Library/motor_control/inc/encoder.h"
#include "Library/motor_control/inc/motor_driver.h"
#include "Library/motor_control/inc/PID_ctl.h"
#include "Library/motor_control/inc/motor_ctl.h"
```

### Configuration:
Edit file paths in your IDE/build system to include:
- `Library/motor_control/inc/` - For headers
- `Library/motor_control/config/` - For configuration
- `Library/motor_control/src/` - For implementation

## Build System Integration

### For STM32CubeMX / Makefile:
```makefile
# Add to include paths
CFLAGS += -ILibrary/motor_control/inc
CFLAGS += -ILibrary/motor_control/config

# Add source files to build
SOURCES += Library/motor_control/src/encoder.cpp
SOURCES += Library/motor_control/src/PID_ctl.cpp
SOURCES += Library/motor_control/src/motor_driver.cpp
SOURCES += Library/motor_control/src/motor_ctl.cpp
```

### For CMake:
```cmake
# Add to CMakeLists.txt
include_directories(Library/motor_control/inc)
include_directories(Library/motor_control/config)

add_library(motor_control
    Library/motor_control/src/encoder.cpp
    Library/motor_control/src/PID_ctl.cpp
    Library/motor_control/src/motor_driver.cpp
    Library/motor_control/src/motor_ctl.cpp
)
```

## Folder Responsibilities

| Folder | Purpose | Editability |
|--------|---------|-------------|
| `inc/` | Public API interfaces | Min changes |
| `src/` | Implementation details | Implementation bugs |
| `config/` | Project-specific parameters | Frequently edit |
| `examples/` | Usage demonstrations | Reference only |
| `docs/` | Documentation | Documentation only |

---

**Tips:**
- Only modify `config/motor_config.h` for parameter tuning
- Don't modify examples unless debugging
- Keep headers in `inc/` for clean API
- All implementation in `src/`
