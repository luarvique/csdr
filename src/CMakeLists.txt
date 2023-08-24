# Copyright (c) 2021-2023 Jakob Ketterl <jakob.ketterl@gmx.de>
#
# This file is part of libcsdr.
#
# libcsdr is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# libcsdr is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with libcsdr.  If not, see <https://www.gnu.org/licenses/>.

configure_file(csdr.pc.in csdr.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/csdr.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

install(EXPORT CsdrTargets
    FILE CsdrTargets.cmake
    NAMESPACE Csdr::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Csdr
)

include(CMakePackageConfigHelpers)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/CsdrConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Csdr
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/CsdrConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/CsdrConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/CsdrConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Csdr
)

add_subdirectory(lib)
add_subdirectory(apps)