/*
 *  Copyright (c) 2011 Evgeny Proydakov <lord.tiran@gmail.com>
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 */

#include <cv_viewer.h>

#include <landscape_generator.h>

// map configure
static const int map_width = 1366;
static const int map_height = 635;

static const int pixel_size = 3;

static const int map_lowland = 0;
static const int map_hill = 255;

int main(int argc, char** argv)
{ 
    (void)argc;
    (void)argv;
    
    cv_viewer::task_set task;
    task.insert(&generator_mounts);
    task.insert(&two_dimensional_random_generator);
    task.insert(&generator_hexagonal_world);
    task.insert(&generator_chess_world);
    task.insert(&generator_triangle_world);
    
    cv_viewer::cv_viewer viewer(map_height, map_width, map_lowland, map_hill, pixel_size);
    viewer.set_task(task);
    viewer.start("landscape");
    viewer.stop();
    
    return 0;
}
