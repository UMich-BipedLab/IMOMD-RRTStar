echo "Building IMOMD..."
g++ -Wall -Wextra -Wno-comment -O3 -std=c++11 \
    -Iinclude \
    main.cpp src/imomd_rrt_star.cpp src/eci_gen_tsp_solver.cpp src/osm_parser.cpp \
    src/baseline/bi_a_star.cpp src/baseline/ana_star.cpp\
    src/tinyxml2/tinyxml2.cpp -lpthread -o main;
