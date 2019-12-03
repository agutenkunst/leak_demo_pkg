# Leak Demo Pkg
This package shows the memory leak of the pluginlib.

Related to
- https://github.com/ros/pluginlib/issues/126
- https://github.com/ros/class_loader/issues/131

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

