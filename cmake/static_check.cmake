# 这行是关键：告诉 CMake 生成编译数据库
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_program(CPPCHECK_EXECUTABLE cppcheck)
if(NOT CPPCHECK_EXECUTABLE)
    message(FATAL_ERROR "Cppcheck executable not found. Please install Cppcheck or set its path.")
endif()

set(CPPCHECK_XML_REPORT ${CMAKE_BINARY_DIR}/cppcheck_report.xml)
add_custom_target(run_cppcheck
    COMMAND ${CPPCHECK_EXECUTABLE}
            --enable=all
            --language=c++
            --std=c++17
            --inline-suppr
            --inconclusive
            --suppress=missingInclude
            --quiet
            --force
            --project=${CMAKE_BINARY_DIR}/compile_commands.json # 使用编译数据库
            --xml
            --xml-version=2
            --output-file=${CPPCHECK_XML_REPORT}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running Cppcheck and generating XML report..."
    VERBATIM
)

find_program(XSLTPROC_EXECUTABLE xsltproc)
if(NOT XSLTPROC_EXECUTABLE)
    message(WARNING "xsltproc executable not found. HTML report generation might not work.")
    # 如果找不到 xsltproc，你可以选择退出或只发出警告
    message(FATAL_ERROR "xsltproc executable not found. Please install xsltproc.")
endif()

set(CPPCHECK_XSLT_STYLESHEET ${CMAKE_SOURCE_DIR}/xslt/cppcheck_report.xslt)
# 确保 XSLT 样式表存在
if(NOT EXISTS ${CPPCHECK_XSLT_STYLESHEET})
    message(FATAL_ERROR "Cppcheck XSLT stylesheet not found: ${CPPCHECK_XSLT_STYLESHEET}")
endif()

# 定义输出的 HTML 报告文件路径
set(CPPCHECK_HTML_REPORT ${CMAKE_BINARY_DIR}/cppcheck_report.html)

# 创建一个自定义目标来运行 XSLT 转换
add_custom_target(generate_cppcheck_html
    # 只有当 xsltproc 可用时才执行命令
    # 如果 xsltproc_executable 为空，COMMAND 列表将为空，这个目标不会执行任何操作
    COMMAND ${XSLTPROC_EXECUTABLE} ${CPPCHECK_XSLT_STYLESHEET} ${CPPCHECK_XML_REPORT} > ${CPPCHECK_HTML_REPORT}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Transforming Cppcheck XML to HTML report..."
    VERBATIM
)

# 让 HTML 报告的生成依赖于 XML 报告的生成
add_dependencies(generate_cppcheck_html run_cppcheck)

# 可选：创建一个all_checks目标，一次性运行cppcheck和生成html
add_custom_target(all_cppcheck_reports)
add_dependencies(all_cppcheck_reports generate_cppcheck_html)