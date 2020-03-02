# add_executable(locclient 
#     main.cpp 
# )

# if(PROFILE)
#     target_compile_definitions(locclient PRIVATE PROFILE=1)
# endif(PROFILE)

# if(HEADLESS)
#     target_compile_definitions(locclient PRIVATE HEADLESS=1)
# endif(HEADLESS)

# target_link_libraries(locclient
#     pthread
#     localiser
#     rtstream
#     rt
#     ${OpenCV_LIBS}
# )

# install(
#         TARGETS locclient
#         RUNTIME DESTINATION bin
#         )