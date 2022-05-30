/* Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/)
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 *
 * AUTHOR: Bruce JK Huang (bjhuang@umich.edu)
 * WEBSITE: https://www.BrucebotStudio.com/
 */
#ifndef UTILS_H
#define UTILS_H 

#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <sstream>      // std::ostringstream
#include <math.h> // M_PI

#include <numeric> // std::iota
#include <vector>
#include <mutex> // std::once_flag, std::call_once


// for time and date
#include <iomanip>
#include <ctime>
#include <sstream>


// print path
#include <unistd.h>
#include <stdio.h>
#include <limits.h>


// #include <algorithm> // std::generate



namespace bipedlab
{
namespace utils
{
    // return a double between [min, max], not [min, max)
    inline double 
    genInclusiveRandomNumber(double min, double max) 
    {
        // run the seed only ONCE
        std::once_flag once_flag;
        std::call_once(once_flag, []{
            srand((unsigned)(time(NULL)));
            return true;
        });
        return min + (max - min) * ((double)rand()/RAND_MAX);
        // return  min + (max - min) * drand48();
    }

    // return a double between [min, max), not [min, max]
    inline double 
    genRandomNumber(double min, double max) 
    {
        // run the seed only ONCE
        std::once_flag once_flag;
        std::call_once(once_flag, []{
            srand((unsigned)(time(NULL)));
            return true;
        });

        srand48((unsigned)(time(NULL)));
        return  min + (max - min) * drand48();
    }

    template <typename T>
        std::vector<T> genListOfInclusiveRandomNumbers(
            int num_of_numbers, double min, double max)
    {
        if (num_of_numbers == 1)
        {
            double num = utils::genInclusiveRandomNumber(min, max);
            std::vector<T> list_of_numbers(num_of_numbers, num);
            return list_of_numbers;
        }
        else
        {
            std::vector<T> list_of_numbers(num_of_numbers, 0);
            for (T& num : list_of_numbers)
            {
                num = utils::genInclusiveRandomNumber(min, max);
            }
            // std::generate(list_of_numbers.begin(), 
            //               list_of_numbers.end(), 
            //               utils::genInclusiveRandomNumber(min, max));
            return list_of_numbers;
        }
    }

    template <typename T>
    std::vector<T> genListOfNumbers(
            const T& start, 
            const size_t num_of_numbers,
            const T& increment)
    {
        std::vector<T> list_of_numbers(num_of_numbers, 0);
        for (int i = 0; i < num_of_numbers; ++i)
        {
            list_of_numbers[i] = i * increment + start;
        }

        return list_of_numbers;
    }

    inline void 
    pressEnterToContinue() {
        signal(SIGINT, [](int signum) {
                std::exit(signum);} // end of lambda expression
        );
        printf("Press [Enter] key to continue.\n");
        while(1)
        {
            char input = getchar();
            if (input == '\n')
                break;
        }; // option TWO to clean stdin
        //getchar(); // wait for ENTER
    }

    template <typename T>
    std::string toStringWithPrecision(const T a_value, const int n = 6)
    {
        std::ostringstream out;
        out.precision(n);
        out << std::fixed << a_value;
        return out.str();
    }


    template <class T>
    double getAverage(const T& this_list)
    {
        double ave = 0;
        for (const auto& element : this_list)
        {
            ave += element;
        }

        return ave / this_list.size();
    }


    inline
    std::string getTimeNDate()
    {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        std::ostringstream oss;
        // oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
        oss << std::put_time(&tm, "%Y-%d-%m-%H-%M-%S");

        return oss.str();
    }


    inline
    std::string getCurrentDirectory()
    {
        char path[PATH_MAX];
        if (getcwd(path, sizeof(path)) == NULL) {
            perror("getcwd() error");
        }

        // if (getcwd(path, sizeof(path)) != NULL) {
        //     printf("Current working directory : %s\n", path);
        // } else {
        //     perror("getcwd() error");
        // }


        return std::string(path);
    }
} /* general */ 
    
} /* bipedlab */ 
#endif /* ifndef UTILS_H */
