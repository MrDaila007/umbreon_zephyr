#pragma once

#if __has_include(<version_git.h>)
#include <version_git.h>
#else
#define FW_GIT_HASH "nogit"
#define FW_COMMIT_SUBJECT "unknown"
#endif

#define FW_VERSION "2.0.3"
#define FW_VERSION_FULL FW_VERSION " (" FW_GIT_HASH ")"
