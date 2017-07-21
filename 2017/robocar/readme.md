### robocar

version 0.0.0 in development


### dependency

- C++14 Standard Library
- Boost C++ Libraries 1.58.0
- OpenCV 3.2.0


### robocar::chrono::for_duration

テンプレート関数．一定時間ループを回したい時にどうぞ．
第一引数に時間（std::chrono::secondsなど），第二引数にファンクタを取ります．
ファンクタには引数として経過時間と継続時間が渡されますので，
ジェネリックラムダを使うのが一番楽だと思います．

使用例：

```
robocar::chrono::for_duration(std::chrono::seconds {5}, [](auto&& elapsed, auto&& duration)
{
  std::cout << "\r\e[K[debug] please wait for " << duration.count() - elapsed.count() << " sec" << std::flush;
  std::this_thread::sleep_for(std::chrono::seconds {1});
});
```

