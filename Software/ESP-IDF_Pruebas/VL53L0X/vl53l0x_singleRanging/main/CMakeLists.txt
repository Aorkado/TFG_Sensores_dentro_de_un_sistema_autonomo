idf_component_register(SRCS "vl53l0x_singleRanging.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver
	    	    PRIV_REQUIRES components)

component_compile_options(-Wno-error=format= -Wno-format)
