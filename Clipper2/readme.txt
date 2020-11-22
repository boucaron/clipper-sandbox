
This is a preview of a major Clipper update. While the code in previous versions 
was functional, in many places it is/was downright ugly. This new version is a 
significant rewrite that should be easier to understand and maintain. There's 
also some performance improvement.

There are many changes to Clipper's interface, and they are too numerous to
mention here. However, notable changes include:

1. The PolyFillType enumeration has been renamed FillRule (TFillRule in Delphi).
2. The cInt type has been replaced with the native 64bit integer type (Int64 
   or int64_t) as there's no longer a need to restrict the range of 64bit path 
   coordinates (at least in the core/base Clipper class).
3. The IntPoint and IntRect types have also been renamed Point64 and Rect64
   respectively (or TPoint64 and TRect64 in Delphi).
4. The Clipper object's Execute parameters have changed with the addition of an
   (optional) OpenPaths (solution) parameter, and with the removal of one of 
   two FillRule parameters. (On reflection, it was probably more confusing 
   than helpful accepting different fill rules for Subject and Clip paths.)
5. The Polytree class now contains only closed paths (ie polygons) since open 
   paths can't contain polygons. (Open paths in the solution are now returned 
   via a separate parameter in Clipper's Execute method.)

   
When I originally translated this Library from Delphi (Pascal) to C# and C++,
I deliberately kept a strong Delphi naming style as I thought this would help
with maintenance. In hindsight this was a mistake. It didn't really achieve
that goal, and it made the C# and C++ code look odd. With this new version, 
I've attempted to adopt a more conventional naming style for each language, 
while admitting that I still have very limited coding experience in both 
these languages.
