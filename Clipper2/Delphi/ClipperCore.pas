unit ClipperCore;

(*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Version   :  10.0 (beta)                                                     *
* Date      :  9 March 2019                                                    *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2019                                         *
* Purpose   :  Core Clipper Library module                                     *
*              Contains structures and functions used throughout the library   *
* License   :  http://www.boost.org/LICENSE_1_0.txt                            *
*******************************************************************************)

{$IFDEF FPC}
  {$DEFINE INLINING}
{$ELSE}
  {$IF CompilerVersion < 14}
    Requires Delphi version 6 or above.
  {$IFEND}
  {$IF CompilerVersion >= 18}         //Delphi 2007
    //While Inlining has been supported since D2005, both D2005 and D2006
    //have an Inline codegen bug (QC41166) so ignore Inline until D2007.
    {$DEFINE INLINING}
    {$IF CompilerVersion >= 25.0}     //Delphi XE4+
      {$LEGACYIFEND ON}
    {$IFEND}
  {$IFEND}
{$ENDIF}

{$IFDEF DEBUG}
  {$UNDEF INLINING}
{$ENDIF}

interface

uses
  Classes, SysUtils, Math;

type
  TPoint64 = record X, Y: Int64; end;
  TPointD = record X, Y: double; end;

  //TPath: a simple data structure to represent a series of vertices, whether
  //open (poly-line) or closed (polygon). A path may be simple or complex (self
  //intersecting). For simple polygons, path orientation (whether clockwise or
  //counter-clockwise) is generally used to differentiate outer paths from inner
  //paths (holes). For complex polygons (and also for overlapping polygons),
  //explicit 'filling rules' (see below) are used to indicate regions that are
  //inside (filled) and regions that are outside (unfilled) a specific polygon.
  TPath = array of TPoint64;
  TPaths = array of TPath;
  TArrayOfPaths = array of TPaths;

  TPathD = array of TPointD;
  TPathsD = array of TPathD;
  TArrayOfPathsD = array of TPathsD;

  TRect64 = {$IFDEF UNICODE}record{$ELSE}object{$ENDIF}
  private
    function GetWidth: Int64; {$IFDEF INLINING} inline; {$ENDIF}
    function GetHeight: Int64; {$IFDEF INLINING} inline; {$ENDIF}
    function GetIsEmpty: Boolean; {$IFDEF INLINING} inline; {$ENDIF}
  public
    Left   : Int64;
    Top    : Int64;
    Right  : Int64;
    Bottom : Int64;
    property Width: Int64 read GetWidth;
    property Height: Int64 read GetHeight;
    property IsEmpty: Boolean read GetIsEmpty;
  end;

  TRectD = {$ifdef UNICODE}record{$else}object{$endif}
  private
    function GetWidth: double; {$IFDEF INLINING} inline; {$ENDIF}
    function GetHeight: double; {$IFDEF INLINING} inline; {$ENDIF}
    function GetIsEmpty: Boolean; {$IFDEF INLINING} inline; {$ENDIF}
  public
    Left   : double;
    Top    : double;
    Right  : double;
    Bottom : double;
    property Width: double read GetWidth;
    property Height: double read GetHeight;
    property IsEmpty: Boolean read GetIsEmpty;
  end;

  TClipType = (ctNone, ctIntersection, ctUnion, ctDifference, ctXor);
  //Note: all clipping operations except for Difference are commutative.
  TPathType = (ptSubject, ptClip);
  //By far the most widely used filling rules for polygons are EvenOdd
  //and NonZero, sometimes called Alternate and Winding respectively.
  //https://en.wikipedia.org/wiki/Nonzero-rule
  TFillRule = (frEvenOdd, frNonZero, frPositive, frNegative);
  TPointInPolygonResult = (pipInside, pipOutside, pipOn);

  EClipperLibException = class(Exception);

//Area: returns type double to avoid potential integer overflows
function Area(const path: TPath): Double; overload;
function Area(const path: TPathD): Double; overload;
function Orientation(const path: TPath): Boolean;
function PointInPolygon(const pt: TPoint64;
  const path: TPath): TPointInPolygonResult;
function CrossProduct(const pt1, pt2, pt3: TPoint64): double;

function PointsEqual(const p1, p2: TPoint64): Boolean; overload;
function PointsEqual(const p1, p2: TPointD): Boolean; overload;
  {$IFDEF INLINING} inline; {$ENDIF}
function Point64(const X, Y: Int64): TPoint64; overload;
  {$IFDEF INLINING} inline; {$ENDIF}
function Point64(const X, Y: Double): TPoint64; overload;
  {$IFDEF INLINING} inline; {$ENDIF}
function PointD(const X, Y: Double): TPointD;
  {$IFDEF INLINING} inline; {$ENDIF}
function Rect64(const left, top, right, bottom: Int64): TRect64; overload;
function Rect64(const recD: TRectD): TRect64; overload;
function RectD(const left, top, right, bottom: double): TRectD; overload;
function RectD(const rec64: TRect64): TRectD; overload;
function GetBounds(const paths: TArrayOfPaths): TRect64; overload;
function GetBounds(const paths: TPaths): TRect64; overload;
function GetBounds(const paths: TPathsD): TRectD; overload;

procedure InflateRect(var rec: TRect64; dx, dy: Int64); overload;
procedure InflateRect(var rec: TRectD; dx, dy: double); overload;
function UnionRect(const rec, rec2: TRect64): TRect64; overload;
function UnionRect(const rec, rec2: TRectD): TRectD; overload;
function RotateRect(const rec: TRect64; angleRad: double): TRect64; overload;
function RotateRect(const rec: TRectD; angleRad: double): TRectD; overload;
procedure OffsetRect(var rec: TRect64; dx, dy: Int64); overload;
procedure OffsetRect(var rec: TRectD; dx, dy: double); overload;

function ScalePath(const path: TPath; sx, sy: double): TPath; overload;
function ScalePath(const path: TPathD; sx, sy: double): TPath; overload;
function ScalePaths(const paths: TPaths; sx, sy: double): TPaths; overload;
function ScalePaths(const paths: TPathsD; sx, sy: double): TPaths; overload;
function ScalePathsD(const paths: TPaths; sx, sy: double): TPathsD; overload;
function ScalePathsD(const paths: TPathsD; sx, sy: double): TPathsD; overload;

function OffsetPath(const path: TPath; dx, dy: Int64): TPath; overload;
function OffsetPath(const path: TPathD; dx, dy: double): TPathD; overload;
function OffsetPaths(const paths: TPaths; dx, dy: Int64): TPaths; overload;
function OffsetPaths(const paths: TPathsD; dx, dy: double): TPathsD; overload;

function Paths(const pathsD: TPathsD): TPaths;
function PathsD(const paths: TPaths): TPathsD;

procedure StripDuplicates(var path: TPath); overload;
procedure StripDuplicates(var path: TPathD); overload;

function ReversePath(const path: TPath): TPath; overload;
function ReversePath(const path: TPathD): TPathD; overload;
function ReversePaths(const paths: TPaths): TPaths; overload;
function ReversePaths(const paths: TPathsD): TPathsD; overload;

procedure AppendPoint(var path: TPath; const pt: TPoint64); overload;
procedure AppendPoint(var path: TPathD; const pt: TPointD); overload;

procedure AppendPath(var paths: TPaths; const extra: TPath); overload;
procedure AppendPath(var paths: TPathsD; const extra: TPathD); overload;
procedure AppendPaths(var paths: TPaths; const extra: TPaths); overload;
procedure AppendPaths(var paths: TPathsD; const extra: TPathsD); overload;

function ArrayOfPathsToPaths(const ap: TArrayOfPaths): TPaths;

//useful debugging functions ...
function PathToString(const path: TPath): string; overload;
function PathsToString(const paths: TPaths): string; overload;
function PathToString(const path: TPathD): string; overload;
function PathsToString(const paths: TPathsD): string; overload;
procedure StringToFile(const str, filename: string);

const
  MinInt64 = -9223372036854775807;
  MaxInt64 = 9223372036854775807;
  NullRect64: TRect64 =
    (left: MaxInt64; top: MaxInt64; right: MinInt64; Bottom: MinInt64);
  NullRectD: TRectD =
    (left: MaxDouble; top: MaxDouble; right: -MaxDouble; Bottom: -MaxDouble);

implementation

//------------------------------------------------------------------------------
// TRect64 methods ...
//------------------------------------------------------------------------------

function TRect64.GetWidth: Int64;
begin
  result := right - left;
end;
//------------------------------------------------------------------------------

function TRect64.GetHeight: Int64;
begin
  result := bottom - top;
end;
//------------------------------------------------------------------------------

function TRect64.GetIsEmpty: Boolean;
begin
  result := (bottom <= top) or (right <= left);
end;

//------------------------------------------------------------------------------
// TRectD methods ...
//------------------------------------------------------------------------------

function TRectD.GetWidth: double;
begin
  result := right - left;
end;
//------------------------------------------------------------------------------

function TRectD.GetHeight: double;
begin
  result := bottom - top;
end;
//------------------------------------------------------------------------------

function TRectD.GetIsEmpty: Boolean;
begin
  result := (bottom <= top) or (right <= left);
end;

//------------------------------------------------------------------------------
// Miscellaneous Functions ...
//------------------------------------------------------------------------------

procedure RaiseError(const msg: string); {$IFDEF INLINING} inline; {$ENDIF}
begin
  raise EClipperLibException.Create(msg);
end;
//------------------------------------------------------------------------------

procedure StripDuplicates(var path: TPath); overload;
var
  i, len: integer;
begin
  len := length(path);
  i := 1;
  while i < len do
    if PointsEqual(path[i], path[i-1]) then
    begin
      dec(len);
      if (i < len) then
        Move(path[i+1], path[i], (len-i)*SizeOf(TPoint64));
      SetLength(path, len);
    end else
      inc(i);
end;
//------------------------------------------------------------------------------

procedure StripDuplicates(var path: TPathD); overload;
var
  i, len: integer;
begin
  len := length(path);
  i := 1;
  while i < len do
    if PointsEqual(path[i], path[i-1]) then
    begin
      dec(len);
      if (i < len) then
        Move(path[i+1], path[i], (len-i)*SizeOf(TPointD));
      SetLength(path, len);
    end else
      inc(i);
end;
//------------------------------------------------------------------------------

function ScalePath(const path: TPath; sx, sy: double): TPath;
var
  i,len: integer;
begin
  if sx = 0 then sx := 1;
  if sy = 0 then sy := 1;
  len := length(path);
  setlength(result, len);
  for i := 0 to len -1 do
  begin
    result[i].X := Round(path[i].X * sx);
    result[i].Y := Round(path[i].Y * sy);
  end;
  StripDuplicates(result);
end;
//------------------------------------------------------------------------------

function ScalePath(const path: TPathD; sx, sy: double): TPath;
var
  i,len: integer;
begin
  if sx = 0 then sx := 1;
  if sy = 0 then sy := 1;
  len := length(path);
  setlength(result, len);
  for i := 0 to len -1 do
  begin
    result[i].X := Round(path[i].X * sx);
    result[i].Y := Round(path[i].Y * sy);
  end;
  StripDuplicates(result);
end;
//------------------------------------------------------------------------------

function ScalePaths(const paths: TPaths; sx, sy: double): TPaths;
var
  i,len: integer;
begin
  if sx = 0 then sx := 1;
  if sy = 0 then sy := 1;
  len := length(paths);
  setlength(result, len);
  for i := 0 to len -1 do
    result[i] := ScalePath(paths[i], sx, sy);
end;
//------------------------------------------------------------------------------

function ScalePaths(const paths: TPathsD; sx, sy: double): TPaths;
var
  i,len: integer;
begin
  if sx = 0 then sx := 1;
  if sy = 0 then sy := 1;
  len := length(paths);
  setlength(result, len);
  for i := 0 to len -1 do
    result[i] := ScalePath(paths[i], sx, sy);
end;
//------------------------------------------------------------------------------

function ScalePathsD(const paths: TPaths; sx, sy: double): TPathsD;
var
  i,j: integer;
begin
  if sx = 0 then sx := 1;
  if sy = 0 then sy := 1;
  setlength(result, length(paths));
  for i := 0 to high(paths) do
  begin
    setlength(result[i], length(paths[i]));
    for j := 0 to high(paths[i]) do
    begin
      result[i][j].X := paths[i][j].X * sx;
      result[i][j].Y := paths[i][j].Y * sy;
    end;
    StripDuplicates(result[i]);
  end;
end;
//------------------------------------------------------------------------------

function ScalePathsD(const paths: TPathsD; sx, sy: double): TPathsD;
var
  i,j: integer;
begin
  if sx = 0 then sx := 1;
  if sy = 0 then sy := 1;
  setlength(result, length(paths));
  for i := 0 to high(paths) do
  begin
    setlength(result[i], length(paths[i]));
    for j := 0 to high(paths[i]) do
    begin
      result[i][j].X := paths[i][j].X * sx;
      result[i][j].Y := paths[i][j].Y * sy;
    end;
  end;
end;
//------------------------------------------------------------------------------

function OffsetPath(const path: TPath; dx, dy: Int64): TPath;
var
  i: integer;
begin
  if (dx = 0) and (dy = 0) then
  begin
    result := path; //nb: reference counted
    Exit;
  end;

  setlength(result, length(path));
  for i := 0 to high(path) do
  begin
    result[i].X := path[i].X + dx;
    result[i].Y := path[i].Y + dy;
  end;
end;
//------------------------------------------------------------------------------

function OffsetPath(const path: TPathD; dx, dy: double): TPathD;
var
  i: integer;
begin
  if (dx = 0) and (dy = 0) then
  begin
    result := path; //nb: reference counted
    Exit;
  end;

  setlength(result, length(path));
  for i := 0 to high(path) do
  begin
    result[i].X := path[i].X + dx;
    result[i].Y := path[i].Y + dy;
  end;
end;
//------------------------------------------------------------------------------

function OffsetPaths(const paths: TPaths; dx, dy: Int64): TPaths;
var
  i,j: integer;
begin
  if (dx = 0) and (dy = 0) then
  begin
    result := paths; //nb: reference counted
    Exit;
  end;

  setlength(result, length(paths));
  for i := 0 to high(paths) do
  begin
    setlength(result[i], length(paths[i]));
    for j := 0 to high(paths[i]) do
    begin
      result[i][j].X := paths[i][j].X + dx;
      result[i][j].Y := paths[i][j].Y + dy;
    end;
  end;
end;
//------------------------------------------------------------------------------

function OffsetPaths(const paths: TPathsD; dx, dy: double): TPathsD;
var
  i,j: integer;
begin
  setlength(result, length(paths));
  for i := 0 to high(paths) do
  begin
    setlength(result[i], length(paths[i]));
    for j := 0 to high(paths[i]) do
    begin
      result[i][j].X := paths[i][j].X + dx;
      result[i][j].Y := paths[i][j].Y + dy;
    end;
  end;
end;
//------------------------------------------------------------------------------

function Paths(const pathsD: TPathsD): TPaths;
var
  i,j,len,len2: integer;
begin
  len := Length(pathsD);
  setLength(Result, len);
  for i := 0 to len -1 do
  begin
    len2 := Length(pathsD[i]);
    setLength(Result[i], len2);
    for j := 0 to len2 -1 do
    begin
      Result[i][j].X := Round(pathsD[i][j].X);
      Result[i][j].Y := Round(pathsD[i][j].Y);
    end;
  end;
end;
//------------------------------------------------------------------------------

function PathsD(const paths: TPaths): TPathsD;
var
  i,j,len,len2: integer;
begin
  len := Length(paths);
  setLength(Result, len);
  for i := 0 to len -1 do
  begin
    len2 := Length(paths[i]);
    setLength(Result[i], len2);
    for j := 0 to len2 -1 do
    begin
      Result[i][j].X := paths[i][j].X;
      Result[i][j].Y := paths[i][j].Y;
    end;
  end;
end;
//------------------------------------------------------------------------------

function ReversePath(const path: TPath): TPath;
var
  i, highI: Integer;
begin
  highI := high(path);
  SetLength(Result, highI +1);
  for i := 0 to highI do
    Result[i] := path[highI - i];
end;
//------------------------------------------------------------------------------

function ReversePath(const path: TPathD): TPathD;
var
  i, highI: Integer;
begin
  highI := high(path);
  SetLength(Result, highI +1);
  for i := 0 to highI do
    Result[i] := path[highI - i];
end;
//------------------------------------------------------------------------------

function ReversePaths(const paths: TPaths): TPaths;
var
  i, j, highJ: Integer;
begin
  i := length(paths);
  SetLength(Result, i);
  for i := 0 to i -1 do
  begin
    highJ := high(paths[i]);
    SetLength(Result[i], highJ+1);
    for j := 0 to highJ do
      Result[i][j] := paths[i][highJ - j];
  end;
end;
//------------------------------------------------------------------------------

function ReversePaths(const paths: TPathsD): TPathsD;
var
  i, j, highJ: Integer;
begin
  i := length(paths);
  SetLength(Result, i);
  for i := 0 to i -1 do
  begin
    highJ := high(paths[i]);
    SetLength(Result[i], highJ+1);
    for j := 0 to highJ do
      Result[i][j] := paths[i][highJ - j];
  end;
end;
//------------------------------------------------------------------------------

procedure AppendPoint(var path: TPath; const pt: TPoint64);
var
  len: Integer;
begin
  len := length(path);
  SetLength(path, len +1);
  path[len] := pt;
end;
//------------------------------------------------------------------------------

procedure AppendPoint(var path: TPathD; const pt: TPointD);
var
  len: Integer;
begin
  len := length(path);
  SetLength(path, len +1);
  path[len] := pt;
end;
//------------------------------------------------------------------------------

procedure AppendPath(var paths: TPaths; const extra: TPath);
var
  len: Integer;
begin
  len := length(paths);
  SetLength(paths, len +1);
  paths[len] := extra;
end;
//------------------------------------------------------------------------------

procedure AppendPath(var paths: TPathsD; const extra: TPathD);
var
  len: Integer;
begin
  len := length(paths);
  SetLength(paths, len +1);
  paths[len] := extra;
end;
//------------------------------------------------------------------------------

procedure AppendPaths(var paths: TPaths; const extra: TPaths);
var
  i, len1, len2: Integer;
begin
  len1 := length(paths);
  len2 := length(extra);
  SetLength(paths, len1 + len2);
  for i := 0 to len2 -1 do
    paths[len1 + i] := extra[i];
end;
//------------------------------------------------------------------------------

procedure AppendPaths(var paths: TPathsD; const extra: TPathsD);
var
  i, len1, len2: Integer;
begin
  len1 := length(paths);
  len2 := length(extra);
  SetLength(paths, len1 + len2);
  for i := 0 to len2 -1 do
    paths[len1 + i] := extra[i];
end;
//------------------------------------------------------------------------------

function ArrayOfPathsToPaths(const ap: TArrayOfPaths): TPaths;
var
  i,j,k, len, cnt: integer;
begin
  cnt := 0;
  len := length(ap);
  for i := 0 to len -1 do
    inc(cnt, length(ap[i]));
  k := 0;
  setlength(result, cnt);
  for i := 0 to len -1 do
    for j := 0 to length(ap[i]) -1 do
    begin
      result[k] := ap[i][j];
      inc(k);
    end;
end;
//------------------------------------------------------------------------------

function PointsEqual(const p1, p2: TPoint64): Boolean;
  {$IFDEF INLINING} inline; {$ENDIF}
begin
  Result := (p1.X = p2.X) and (p1.Y = p2.Y);
end;
//------------------------------------------------------------------------------

function PointsEqual(const p1, p2: TPointD): Boolean;
  {$IFDEF INLINING} inline; {$ENDIF}
begin
  Result := (p1.X = p2.X) and (p1.Y = p2.Y);
end;
//------------------------------------------------------------------------------

function Point64(const X, Y: Int64): TPoint64;
begin
  Result.X := X;
  Result.Y := Y;
end;
//------------------------------------------------------------------------------

function Point64(const X, Y: Double): TPoint64;
begin
  Result.X := Round(X);
  Result.Y := Round(Y);
end;
//------------------------------------------------------------------------------

function PointD(const X, Y: Double): TPointD;
begin
  Result.X := X;
  Result.Y := Y;
end;
//------------------------------------------------------------------------------

function Rect64(const left, top, right, bottom: Int64): TRect64;
begin
  Result.Left   := left;
  Result.Top    := top;
  Result.Right  := right;
  Result.Bottom := bottom;
end;
//------------------------------------------------------------------------------

function Rect64(const recD: TRectD): TRect64;
begin
  Result.Left   := Floor(recD.left);
  Result.Top    := Floor(recD.top);
  Result.Right  := Ceil(recD.right);
  Result.Bottom := Ceil(recD.bottom);
end;
//------------------------------------------------------------------------------

function RectD(const left, top, right, bottom: double): TRectD;
begin
  Result.Left   := left;
  Result.Top    := top;
  Result.Right  := right;
  Result.Bottom := bottom;
end;
//------------------------------------------------------------------------------

function RectD(const rec64: TRect64): TRectD; overload;
begin
  Result.Left   := rec64.left;
  Result.Top    := rec64.top;
  Result.Right  := rec64.right;
  Result.Bottom := rec64.bottom;
end;
//------------------------------------------------------------------------------

function GetBounds(const paths: TArrayOfPaths): TRect64; overload;
var
  i,j,k: Integer;
begin
  Result := Rect64(High(Int64), High(Int64), Low(Int64), Low(Int64));
  for i := 0 to High(paths) do
    for j := 0 to High(paths[i]) do
      for k := 0 to High(paths[i][j]) do
      begin
        if paths[i][j][k].X < Result.Left then Result.Left := paths[i][j][k].X;
        if paths[i][j][k].X > Result.Right then Result.Right := paths[i][j][k].X;
        if paths[i][j][k].Y < Result.Top then Result.Top := paths[i][j][k].Y;
        if paths[i][j][k].Y > Result.Bottom then Result.Bottom := paths[i][j][k].Y;
      end;
  if Result.Left > Result.Right then Result := nullRect64;
end;
//------------------------------------------------------------------------------

function GetBounds(const paths: TPaths): TRect64;
var
  i,j: Integer;
begin
  Result := Rect64(High(Int64), High(Int64), Low(Int64), Low(Int64));
  for i := 0 to High(paths) do
    for j := 0 to High(paths[i]) do
    begin
      if paths[i][j].X < Result.Left then Result.Left := paths[i][j].X;
      if paths[i][j].X > Result.Right then Result.Right := paths[i][j].X;
      if paths[i][j].Y < Result.Top then Result.Top := paths[i][j].Y;
      if paths[i][j].Y > Result.Bottom then Result.Bottom := paths[i][j].Y;
    end;
  if Result.Left > Result.Right then Result := nullRect64;
end;
//------------------------------------------------------------------------------

function GetBounds(const paths: TPathsD): TRectD;
var
  i,j: Integer;
begin
  Result := RectD(MaxDouble, MaxDouble, -MaxDouble, -MaxDouble);
  for i := 0 to High(paths) do
    for j := 0 to High(paths[i]) do
    begin
      if paths[i][j].X < Result.Left then Result.Left := paths[i][j].X;
      if paths[i][j].X > Result.Right then Result.Right := paths[i][j].X;
      if paths[i][j].Y < Result.Top then Result.Top := paths[i][j].Y;
      if paths[i][j].Y > Result.Bottom then Result.Bottom := paths[i][j].Y;
    end;
  result.Left := Floor(Result.Left);
  result.Top := Floor(Result.Top);
  result.Right := Ceil(Result.Right);
  result.Bottom := Ceil(Result.Bottom);
  if Result.Left >= Result.Right then Result := nullRectD;
end;
//------------------------------------------------------------------------------

procedure InflateRect(var rec: TRect64; dx, dy: Int64);
begin
  dec(rec.Left, dx);
  inc(rec.Right, dx);
  dec(rec.Top, dy);
  inc(rec.Bottom, dy);
end;
//------------------------------------------------------------------------------

procedure InflateRect(var rec: TRectD; dx, dy: double);
begin
  rec.Left := rec.Left - dx;
  rec.Right := rec.Right + dx;
  rec.Top := rec.Top - dy;
  rec.Bottom := rec.Bottom + dy;
end;
//------------------------------------------------------------------------------

procedure RotatePt(var pt: TPointD; const center: TPointD; sinA, cosA: double);
var
  tmpX, tmpY: double;
begin
  tmpX := pt.X-center.X;
  tmpY := pt.Y-center.Y;
  pt.X := tmpX * cosA - tmpY * sinA + center.X;
  pt.Y := tmpX * sinA + tmpY * cosA + center.Y;
end;
//------------------------------------------------------------------------------

function RotateRect(const rec: TRectD; angleRad: double): TRectD;
var
  i: integer;
  sinA, cosA: double;
  cp: TPointD;
  pts: TPathD;
begin
  setLength(pts, 4);
  sinA := Sin(-angleRad);
  cosA := cos(-angleRad);
  cp.X := (rec.Right + rec.Left) / 2;
  cp.Y := (rec.Bottom + rec.Top) / 2;
  pts[0] := PointD(rec.Left, rec.Top);
  pts[1] := PointD(rec.Right, rec.Top);
  pts[2] := PointD(rec.Left, rec.Bottom);
  pts[3] := PointD(rec.Right, rec.Bottom);
  for i := 0 to 3 do RotatePt(pts[i], cp, sinA, cosA);
  result.Left := pts[0].X;
  result.Right := result.Left;
  result.Top := pts[0].Y;
  result.Bottom := result.Top;
  for i := 1 to 3 do
  begin
    if pts[i].X < result.Left then result.Left := pts[i].X;
    if pts[i].Y < result.Top then result.Top := pts[i].Y;
    if pts[i].X > result.Right then result.Right := pts[i].X;
    if pts[i].Y > result.Bottom then result.Bottom := pts[i].Y;
  end;
end;
//------------------------------------------------------------------------------

function RotateRect(const rec: TRect64; angleRad: double): TRect64;
var
  recD: TRectD;
begin
  recD := RectD(rec.Left, rec.Top, rec.Right, rec.Bottom);
  recD := RotateRect(recD, angleRad);
  result.Left := Floor(recD.Left);
  result.Top := Floor(recD.Top);
  result.Right := Ceil(recD.Right);
  result.Bottom := Ceil(recD.Bottom);
end;
//------------------------------------------------------------------------------

procedure OffsetRect(var rec: TRect64; dx, dy: Int64);
begin
  inc(rec.Left, dx); inc(rec.Top, dy);
  inc(rec.Right, dx); inc(rec.Bottom, dy);
end;
//------------------------------------------------------------------------------

procedure OffsetRect(var rec: TRectD; dx, dy: double);
begin
  rec.Left   := rec.Left   + dx;
  rec.Right  := rec.Right  + dx;
  rec.Top    := rec.Top    + dy;
  rec.Bottom := rec.Bottom + dy;
end;
//------------------------------------------------------------------------------

function UnionRect(const rec, rec2: TRect64): TRect64;
begin
  if rec.IsEmpty then result := rec2
  else if rec2.IsEmpty then result := rec
  else
  begin
    result.Left := min(rec.Left, rec2.Left);
    result.Right := max(rec.Right, rec2.Right);
    result.Top := min(rec.Top, rec2.Top);
    result.Bottom := max(rec.Bottom, rec2.Bottom);
  end;
end;
//------------------------------------------------------------------------------

function UnionRect(const rec, rec2: TRectD): TRectD;
begin
  if rec.IsEmpty then result := rec2
  else if rec2.IsEmpty then result := rec
  else
  begin
    result.Left := min(rec.Left, rec2.Left);
    result.Right := max(rec.Right, rec2.Right);
    result.Top := min(rec.Top, rec2.Top);
    result.Bottom := max(rec.Bottom, rec2.Bottom);
  end;
end;
//------------------------------------------------------------------------------

function Area(const path: TPath): Double;
var
  i, j, highI: Integer;
  d: Double;
begin
  Result := 0.0;
  highI := High(path);
  if (highI < 2) then Exit;
  j := highI;
  for i := 0 to highI do
  begin
    d := (path[j].X + path[i].X);
    Result := Result + d * (path[j].Y - path[i].Y);
    j := i;
  end;
  Result := -Result * 0.5;
end;
//------------------------------------------------------------------------------

function Area(const path: TPathD): Double;
var
  i, j, highI: Integer;
  d: Double;
begin
  Result := 0.0;
  highI := High(path);
  if (highI < 2) then Exit;
  j := highI;
  for i := 0 to highI do
  begin
    d := (path[j].X + path[i].X);
    Result := Result + d * (path[j].Y - path[i].Y);
    j := i;
  end;
  Result := -Result * 0.5;
end;
//------------------------------------------------------------------------------

function Orientation(const path: TPath): Boolean;
begin
  Result := Area(path) >= 0;
end;
//------------------------------------------------------------------------------

function PointInPolygon(const pt: TPoint64;
  const path: TPath): TPointInPolygonResult;
var
  i, val, cnt: Integer;
  d, d2, d3: Double; //using doubles to avoid possible integer overflow
  ip, ipNext: TPoint64;
begin
  cnt := Length(path);
  if cnt < 3 then
  begin
    result := pipOutside;
    Exit;
  end;
  ip := path[0];
  Result := pipOn;
  val := 0;
  for i := 1 to cnt do
  begin
    if i < cnt then ipNext := path[i]
    else ipNext := path[0];

    if (ipNext.Y = pt.Y) then
    begin
      if (ipNext.X = pt.X) or ((ip.Y = pt.Y) and
        ((ipNext.X > pt.X) = (ip.X < pt.X))) then Exit;
    end;

    if ((ip.Y < pt.Y) <> (ipNext.Y < pt.Y)) then
    begin
      if (ip.X >= pt.X) then
      begin
        if (ipNext.X > pt.X) then val := 1 - val
        else
        begin
          d2 := (ip.X - pt.X);
          d3 := (ipNext.X - pt.X);
          d := d2 * (ipNext.Y - pt.Y) - d3 * (ip.Y - pt.Y);
          if (d = 0) then Exit;
          if ((d > 0) = (ipNext.Y > ip.Y)) then val := 1 - val;
        end;
      end else
      begin
        if (ipNext.X > pt.X) then
        begin
          d2 := (ip.X - pt.X);
          d3 := (ipNext.X - pt.X);
          d := d2 * (ipNext.Y - pt.Y) - d3 * (ip.Y - pt.Y);
          if (d = 0) then Exit;
          if ((d > 0) = (ipNext.Y > ip.Y)) then val := 1 - val;
        end;
      end;
    end;
    ip := ipNext;
  end;

  case val of
    -1: result := pipOn;
     1: result := pipInside;
     else result := pipOutside;
  end;
end;
//---------------------------------------------------------------------------

function CrossProduct(const pt1, pt2, pt3: TPoint64): double;
var
  x1,x2,y1,y2: double;
begin
  x1 := pt2.X - pt1.X;
  y1 := pt2.Y - pt1.Y;
  x2 := pt3.X - pt2.X;
  y2 := pt3.Y - pt2.Y;
  result := (x1 * y2 - y1 * x2);
end;
//------------------------------------------------------------------------------

function PointTosString(const pt: TPoint64): string;
  {$IFDEF INLINING} inline; {$ENDIF}
begin
  result := format('%d,%d', [pt.X, pt.Y]);
end;
//------------------------------------------------------------------------------

function PathToString(const path: TPath): string;
var
  i, highI: integer;
begin
  result := '';
  highI := high(path);
  if highI < 0 then Exit;
  for i := 0 to highI -1 do
    result := result + PointTosString(path[i]) + ', ';
  result := result + PointTosString(path[highI]);
end;
//------------------------------------------------------------------------------

function PathsToString(const paths: TPaths): string;
var
  i, highI: integer;
begin
  result := '';
  highI := high(paths);
  if highI < 0 then Exit;
  for i := 0 to highI do
    result := result + PathToString(paths[i]) + #10;
end;
//------------------------------------------------------------------------------

function PointDTosString(const pt: TPointD): string;
  {$IFDEF INLINING} inline; {$ENDIF}
begin
  result := format('%1.2f,%1.2f', [pt.X, pt.Y]);
end;
//------------------------------------------------------------------------------

function PathToString(const path: TPathD): string;
var
  i, highI: integer;
begin
  result := '';
  highI := high(path);
  if highI < 0 then Exit;
  for i := 0 to highI -1 do
    result := result + PointDTosString(path[i]) + ', ';
  result := result + PointDTosString(path[highI]);
end;
//------------------------------------------------------------------------------

function PathsToString(const paths: TPathsD): string;
var
  i, highI: integer;
begin
  result := '';
  highI := high(paths);
  if highI < 0 then Exit;
  for i := 0 to highI do
    result := result + PathToString(paths[i]) + #10#10;
end;
//------------------------------------------------------------------------------

procedure StringToFile(const str, filename: string);
begin
  with TStringList.Create do
  try
    Text := str;
    SaveToFile(filename);
  finally
    free;
  end;
end;

//------------------------------------------------------------------------------


end.

