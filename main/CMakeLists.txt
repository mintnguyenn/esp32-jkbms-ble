idf_component_register(SRCS "dashboard_server.cpp" "bms_data_decode.cpp" "ble_manager.cpp" "main.cpp"
                    INCLUDE_DIRS ".")

target_include_directories(${COMPONENT_LIB} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../managed_components/joltwallet__littlefs/src/littlefs"
)