idf_component_register(SRC_DIRS "core/src" "platform/src"
                       INCLUDE_DIRS "core/inc" "platform/inc"
		       REQUIRES driver)
component_compile_options(-Wno-error=format= -Wno-format)
