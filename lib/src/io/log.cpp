#include <rmb/io/log.h>

namespace rmb {

const OutStream& outs() {
    static OutStream os = OutStream();
    return os;
}

} // namespace rmb
