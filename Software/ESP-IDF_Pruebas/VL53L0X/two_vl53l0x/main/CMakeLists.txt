idf_component_register(SRCS "two_vl53l0x.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver
	    	    PRIV_REQUIRES components)

component_compile_options(-Wno-error=format= -Wno-format)
