idf_component_register(SRCS         "ak8963.c"
                                    "common9250.c"
                                    "i2c-easy.c"
                                    "mpu9250.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver)