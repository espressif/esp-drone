idf_component_register(SRCS "buzzdeck.c" "piezo.c"
                       INCLUDE_DIRS "." "include"
                        REQUIRES hal modules utils platform config)