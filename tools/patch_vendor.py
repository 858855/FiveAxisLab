"""Small reproducible build-compatibility patches to pinned upstream sources."""
from pathlib import Path
root = Path(__file__).resolve().parents[1]
changes = {
    'vendor/ompl/src/ompl/CMakeLists.txt': [('target_link_libraries(ompl psapi ws2_32)', 'target_link_libraries(ompl PRIVATE psapi ws2_32)')],
    'vendor/ompl/CMakeLists.txt': [('add_custom_target(uninstall\n', 'add_custom_target(ompl_uninstall\n')],
    'vendor/fcl/CMakeLists.txt': [('find_package(Eigen3 3.0.5 QUIET CONFIG)', 'find_package(Eigen3 QUIET CONFIG)')],
    'vendor/fcl/CMakeModules/CompilerSettings.cmake': [('-std=c++11', '-std=c++17')],
    'vendor/fcl/include/fcl/geometry/shape/convex-inl.h': [('#include <map>', '#include <map>\n#include <cassert>')],
}
for filename, replacements in changes.items():
    path = root / filename
    text = path.read_text(encoding='utf-8')
    original=text
    for old, new in replacements:
        if old in text:
            text = text.replace(old, new)
        elif new not in text:
            raise RuntimeError('Upstream patch context changed: ' + filename)
    if text!=original:path.write_text(text, encoding='utf-8')
for path in (root / 'vendor/fcl').rglob('*'):
    if path.suffix not in ('.h', '.cpp'):
        continue
    text = path.read_text(encoding='utf-8')
    if 'assert(' in text and '#include <cassert>' not in text and '#include <assert.h>' not in text:
        index = text.find('#include')
        if index >= 0:
            path.write_text(text[:index] + '#include <cassert>\n' + text[index:], encoding='utf-8')
