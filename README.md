# Leak Demo Pkg
This package shows the memory leak of the pluginlib.

Related to
- https://github.com/ros/pluginlib/issues/126
- https://github.com/ros/class_loader/issues/131

Maybe this is https://github.com/google/sanitizers/issues/89
https://stackoverflow.com/questions/44627258/addresssanitizer-and-loading-of-dynamic-libraries-at-runtime-unknown-module

## Demonstrate
Run
```
catkin_make -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS_DEBUG="-g -fno-omit-frame-pointer -fsanitize=address" -DCMAKE_EXE_LINKER_FLAGS_DEBUG="-g -fsanitize=address" && rosrun leak_demo_pkg main
```

which results in
```
==26394==ERROR: LeakSanitizer: detected memory leaks

Direct leak of 160 byte(s) in 1 object(s) allocated from:
    #0 0x7fd2c6864458 in operator new(unsigned long) (/usr/lib/x86_64-linux-gnu/libasan.so.4+0xe0458)
    #1 0x7fd2bbdb4257  (<unknown module>)
    #2 0x7fd2bbdb22e8  (<unknown module>)
    #3 0x7fd2bbdb25e2  (<unknown module>)
    #4 0x7fd2bbdb25fd  (<unknown module>)
    #5 0x7fd2c774f732  (/lib64/ld-linux-x86-64.so.2+0x10732)

Indirect leak of 55 byte(s) in 1 object(s) allocated from:
    #0 0x7fd2c6864458 in operator new(unsigned long) (/usr/lib/x86_64-linux-gnu/libasan.so.4+0xe0458)
    #1 0x7fd2c5677bc6 in std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >::_M_assign(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&) (/usr/lib/x86_64-linux-gnu/libstdc++.so.6+0x124bc6)

Indirect leak of 21 byte(s) in 1 object(s) allocated from:
    #0 0x7fd2c6864458 in operator new(unsigned long) (/usr/lib/x86_64-linux-gnu/libasan.so.4+0xe0458)
    #1 0x7fd2c567a3ac in void std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >::_M_construct<char const*>(char const*, char const*, std::forward_iterator_tag) (/usr/lib/x86_64-linux-gnu/libstdc++.so.6+0x1273ac)
    #2 0x7fd2c79284ff  (<unknown module>)
    #3 0x7fd2c792b90f  (<unknown module>)

Indirect leak of 18 byte(s) in 1 object(s) allocated from:
    #0 0x7fd2c6864458 in operator new(unsigned long) (/usr/lib/x86_64-linux-gnu/libasan.so.4+0xe0458)
    #1 0x7fd2c6574b6c  (/opt/ros/melodic/lib/libclass_loader.so+0x17b6c)

Indirect leak of 8 byte(s) in 1 object(s) allocated from:
    #0 0x7fd2c6864458 in operator new(unsigned long) (/usr/lib/x86_64-linux-gnu/libasan.so.4+0xe0458)
    #1 0x7fd2c6575383 in void std::vector<class_loader::ClassLoader*, std::allocator<class_loader::ClassLoader*> >::_M_realloc_insert<class_loader::ClassLoader* const&>(__gnu_cxx::__normal_iterator<class_loader::ClassLoader**, std::vector<class_loader::ClassLoader*, std::allocator<class_loader::ClassLoader*> > >, class_loader::ClassLoader* const&) (/opt/ros/melodic/lib/libclass_loader.so+0x18383)

SUMMARY: AddressSanitizer: 262 byte(s) leaked in 5 allocation(s).
```

## Using valgrind
Running (rebuild without sanitizing flags before!)
```
valgrind --leak-check=full ./main
```

yields
```
==25985== Memcheck, a memory error detector
==25985== Copyright (C) 2002-2017, and GNU GPL'd, by Julian Seward et al.
==25985== Using Valgrind-3.13.0 and LibVEX; rerun with -h for copyright info
==25985== Command: ./main
==25985==
==25985==
==25985== HEAP SUMMARY:
==25985==     in use at exit: 2,957 bytes in 49 blocks
==25985==   total heap usage: 15,970 allocs, 15,921 frees, 18,547,566 bytes allocated
==25985==
==25985== 264 (160 direct, 104 indirect) bytes in 1 blocks are definitely lost in loss record 46 of 46
==25985==    at 0x4C3017F: operator new(unsigned long) (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==25985==    by 0xDCE682C: ???
==25985==    by 0xDCE5BC7: ???
==25985==    by 0xDCE5CC4: ???
==25985==    by 0xDCE5CDA: ???
==25985==    by 0x4010732: call_init (dl-init.c:72)
==25985==    by 0x4010732: _dl_init (dl-init.c:119)
==25985==    by 0x40151FE: dl_open_worker (dl-open.c:522)
==25985==    by 0x660B2DE: _dl_catch_exception (dl-error-skeleton.c:196)
==25985==    by 0x40147C9: _dl_open (dl-open.c:605)
==25985==    by 0x7F2DF95: dlopen_doit (dlopen.c:66)
==25985==    by 0x660B2DE: _dl_catch_exception (dl-error-skeleton.c:196)
==25985==    by 0x660B36E: _dl_catch_error (dl-error-skeleton.c:215)
==25985==
==25985== LEAK SUMMARY:
==25985==    definitely lost: 160 bytes in 1 blocks
==25985==    indirectly lost: 104 bytes in 4 blocks
==25985==      possibly lost: 0 bytes in 0 blocks
==25985==    still reachable: 2,693 bytes in 44 blocks
==25985==         suppressed: 0 bytes in 0 blocks
==25985== Reachable blocks (those to which a pointer was found) are not shown.
==25985== To see them, rerun with: --leak-check=full --show-leak-kinds=all
==25985==
==25985== For counts of detected and suppressed errors, rerun with: -v
==25985== ERROR SUMMARY: 1 errors from 1 contexts (suppressed: 0 from 0)
```

where the **160 bytes** also show up.