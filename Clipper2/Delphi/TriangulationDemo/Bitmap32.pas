unit Bitmap32;

(*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Version   :  1.0                                                             *
* Date      :  24 January 2019                                                 *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2019                                         *
* Purpose   :  Module to maniputlate 32bit images                              *
* License   :  http://www.boost.org/LICENSE_1_0.txt                            *
*******************************************************************************)

{$IFDEF DEBUG}
  {$UNDEF INLINING}
{$ENDIF}

interface

uses
  SysUtils, Classes, Windows, Math;

type
  PColor32 = ^TColor32;
  TColor32 = Cardinal;

  PColor32Array = ^TColor32Array;
  TColor32Array = array of TColor32;
  TArrayOfInteger = array of integer;
  TArrayOfByte = array of byte;

  TBitmap32ExtClass = class of TBitmap32Ext;
  TBitmap32 = class;

  //TBitmap32Ext abstract base class that templates
  //access to multiple image file formats ...
  TBitmap32Ext = class
    class function SaveToFile(const filename: string;
      bmp32: TBitmap32): Boolean; virtual; abstract;
    class function LoadFromFile(const filename: string;
      bmp32: TBitmap32): Boolean; virtual; abstract;
  end;

  TBitmap32 = class
  private
    fPixels: TColor32Array;
    fAntiAliase: Boolean;
    fWidth: integer;
    fHeight: integer;
    function GetPixel(x,y: integer): TColor32;
      {$IFDEF INLINING} inline; {$ENDIF}
    function GetPixelRow(index: integer): PColor32;
    procedure SetPixel(x,y: integer; color: TColor32);
    function GetWeightedPixel(x256, y256: integer): TColor32;
    function GetIsEmpty: Boolean; {$IFDEF INLINING} inline; {$ENDIF}
    function GetPixelBase: PColor32;
    function DoResizeAA(newWidth, newHeight: integer): TColor32Array;
    function DoResize(newWidth, newHeight: integer): TColor32Array;
    function CountColors: integer;
  public
    constructor Create(width: integer = 0;
      height: integer = 0; bits: Pointer = nil); virtual;
    destructor Destroy; override;
    class procedure RegisterExtension(const ext: string;
      bm32ExClass: TBitmap32ExtClass);
    procedure Fill(color: TColor32); {$IFDEF INLINING} inline; {$ENDIF}
    procedure FillRect(rec: TRect; color: TColor32);
    procedure Insert(x,y: integer; image: TBitmap32);
    procedure Crop(rec: TRect);
    procedure CropTransparentPixels;
    procedure AssignTo(dst: TBitmap32);
    procedure DrawTo(dst: TBitmap32);          //scales image to 'dst' size
    procedure SetSize(width, height: integer; zeroFill: Boolean = true);
    procedure Resize(width, height: integer);  //scales image
    procedure Scale(value: single);            //scales maintaining proportions
    procedure Rotate(angleRads: single);
    procedure FlipVertical;
    procedure FlipHorizontal;

    //SaveToFile: requires a TBitmap32Ext - see RegisterExtension() above
    function SaveToFile(const filename: string): Boolean;
    //LoadFromFile: requires a TBitmap32Ext - see RegisterExtension() above
    function LoadFromFile(const filename: string): Boolean;

    //properties ...
    property Width: integer read fWidth;
    property Height: integer read fHeight;
    property IsEmpty: Boolean read GetIsEmpty;
    property Pixel[x,y: integer]: TColor32 read GetPixel write SetPixel;
    property Pixels: TColor32Array read fPixels;
    property PixelBase: PColor32 read GetPixelBase;
    property PixelRow[idx: integer]: PColor32 read GetPixelRow;
    property ColorCount: integer read CountColors;
    property EnableAntiAliase: Boolean read fAntiAliase write fAntiAliase;
  end;

  PARGB = ^TARGB;
  TARGB = packed record
    case boolean of
      false: (B: Byte; G: Byte; R: Byte; A: Byte);
      true : (Color: TColor32);
  end;

  PTriColor32 = ^TTriColor32;
  TTriColor32 = array [0..2] of TColor32;

  function MakeLighter(color: TColor32; percent: integer): TColor32;
  function MakeDarker(color: TColor32; percent: integer): TColor32;

const
  clAqua32     = TColor32($FF00FFFF);
  clBlack32    = TColor32($FF000000);
  clBlue32     = TColor32($FF0000FF);
  clFuchsia32  = TColor32($FFFF00FF);
  clGray32     = TColor32($FF808080);
  clGreen32    = TColor32($FF008000);
  clLime32     = TColor32($FF00FF00);
  clMaroon32   = TColor32($FF000080);
  clNavy32     = TColor32($FF000080);
  clNone32     = TColor32($00000000);
  clRed32      = TColor32($FFFF0000);
  clSilver32   = TColor32($FFC0C0C0);
  clWhite32    = TColor32($FFFFFFFF);
  clYellow32   = TColor32($FFFFFF00);

implementation

type

  TColor24 = packed record R,G,B: byte; end;
  TColor24Array = array of TColor24;

  TWeightedColor = {$IFDEF UNICODE}record{$ELSE}object{$ENDIF}
  private
    fAddCount : cardinal;
    fAlphaTot : cardinal;
    fColorTotR: cardinal;
    fColorTotG: cardinal;
    fColorTotB: cardinal;
    function GetColor: TColor32;
  public
    procedure Reset;
    procedure AddWeight(weight: cardinal);
    procedure Add(c: TColor32; weight: cardinal);
    property AddCount: cardinal read fAddCount;
    property Color: TColor32 read GetColor;
  end;

  TPointD = record X, Y: double; end;
  TPathD = array of TPointD;
  TRectD = record Left, Top, Right, Bottom: double; end;

var
  Bitmap32ExtClassList: TStringList;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

function MakeLighter(color: TColor32; percent: integer): TColor32;
var
  src: TARGB absolute color;
  dst: TARGB absolute result;
begin
  //ideally we'd convert color to HSL format before adjusting luminence
  //but for most purposes the following is close enough :)
  if percent < 0 then percent := 0
  else if percent > 100 then percent := 100;
  dst.A := src.A;
  dst.R := src.R + MulDiv(255 - src.R, percent, 100);
  dst.G := src.G + MulDiv(255 - src.G, percent, 100);
  dst.B := src.B + MulDiv(255 - src.B, percent, 100);
end;
//------------------------------------------------------------------------------

function MakeDarker(color: TColor32; percent: integer): TColor32;
var
  src: TARGB absolute color;
  dst: TARGB absolute result;
begin
  //ideally we'd convert color to HSL format before adjusting luminence
  //but for most purposes the following is close enough :)
  if percent < 0 then percent := 0
  else if percent > 100 then percent := 100;
  dst.A := src.A;
  dst.R := src.R - MulDiv(src.R, percent, 100);
  dst.G := src.G - MulDiv(src.G, percent, 100);
  dst.B := src.B - MulDiv(src.B, percent, 100);
end;
//------------------------------------------------------------------------------

function GetWeightedColor(const srcBits: TColor32Array;
  x256, y256, xx256, yy256, maxX: integer): TColor32;
var
  i, j, xi, yi, xxi, yyi, weight: integer;
  xf, yf, xxf, yyf: cardinal;
  color: TWeightedColor;
begin
  color.Reset;

  xi := x256 shr 8; xf := x256 and $FF;
  yi := y256 shr 8; yf := y256 and $FF;
  xxi := xx256 shr 8; xxf := xx256 and $FF;
  yyi := yy256 shr 8; yyf := yy256 and $FF;

  //1. average the corners ...

  weight := (($100 - xf) * ($100 - yf)) shr 8;
  color.Add(srcBits[xi + yi * maxX], weight);
  weight := (xxf * ($100 - yf)) shr 8;
  if (weight <> 0) then color.Add(srcBits[xxi + yi * maxX], weight);
  weight := (($100 - xf) * yyf) shr 8;
  if (weight <> 0) then color.Add(srcBits[xi + yyi * maxX], weight);
  weight := (xxf * yyf) shr 8;
  if (weight <> 0) then color.Add(srcBits[xxi + yyi * maxX], weight);

  //2. average the edges
  if (yi +1 < yyi) then
  begin
    xf := $100 - xf;
    for i := yi + 1 to yyi - 1 do
      color.Add(srcBits[xi + i * maxX], xf);
    if (xxf <> 0) then
      for i := yi + 1 to yyi - 1 do
        color.Add(srcBits[xxi + i * maxX], xxf);
  end;
  if (xi + 1 < xxi) then
  begin
    yf := $100 - yf;
    for i := xi + 1 to xxi - 1 do
      color.Add(srcBits[i + yi * maxX], yf);
    if (yyf <> 0) then
      for i := xi + 1 to xxi - 1 do
        color.Add(srcBits[i + yyi * maxX], yyf);
  end;

  //3. average the non-fractional pixel 'internals' ...
  for i := xi + 1 to xxi - 1 do
    for j := yi + 1 to yyi - 1 do
      color.Add(srcBits[i + j * maxX], $100);

  if color.AddCount = 0 then
    Result := srcBits[xi + yi * maxX] else
    Result := color.Color;
end;

//------------------------------------------------------------------------------
// TBitmap32 methods
//------------------------------------------------------------------------------

constructor TBitmap32.Create(width: integer = 0;
  height: integer = 0; bits: Pointer = nil);
begin
  fAntiAliase := true;
  SetSize(width, height, bits = nil);
  if (width > 0) and (height > 0) and assigned(bits) then
    Move(bits^, fPixels[0], width * height * sizeof(TColor32));
end;
//------------------------------------------------------------------------------

destructor TBitmap32.Destroy;
begin
  fPixels := nil;
  inherited;
end;
//------------------------------------------------------------------------------

class procedure TBitmap32.RegisterExtension(const ext: string;
  bm32ExClass: TBitmap32ExtClass);
var
  idx: integer;
  ex: string;
begin
  ex := lowercase(ext);
  if ex = '' then Exit
  else if ext[1] = '.' then Delete(ex, 1,1);
  if Bitmap32ExtClassList.Find(ex, idx) then Exit;
  Bitmap32ExtClassList.AddObject(ex, Pointer(bm32ExClass));
end;
//------------------------------------------------------------------------------

procedure TBitmap32.Fill(color: TColor32);
var
  i: integer;
begin
  for i := Width * Height -1 downto 0 do fPixels[i] := color;
end;
//------------------------------------------------------------------------------

procedure TBitmap32.FillRect(rec: TRect; color: TColor32);
var
  i,j: integer;
  c: PColor32;
begin
  if rec.Left < 0 then rec.Left := 0;
  if rec.Right > Width then rec.Right := Width;
  if rec.Top < 0 then rec.Top := 0;
  if rec.Bottom > Height then rec.Bottom := Height;
  if (rec.Right <= rec.Left) or (rec.Bottom <= rec.Top) then Exit;

  for i := Height-rec.Bottom to Height-rec.Top-1 do
  begin
    c := @Pixels[i * Width + rec.Left];
    for j := 1 to rec.Right-rec.Left do
    begin
      c^ := color;
      inc(c);
    end;
  end;
end;
//------------------------------------------------------------------------------

{$IFNDEF UNICODE}
function IsRectEmpty(const Rect: TRect): Boolean;
begin
  Result := (Rect.Right <= Rect.Left) or (Rect.Bottom <= Rect.Top);
end;
{$ENDIF}
//------------------------------------------------------------------------------

procedure TBitmap32.Insert(x,y: integer; image: TBitmap32);
var
  i, j: integer;
  c: PColor32;
  srcRec, dstRec: TRect;
begin
  srcRec := Rect(0, 0, image.Width, image.Height);
  if x < 0 then inc(srcRec.Left, -x);
  if y < 0 then inc(srcRec.Top, -y);
{$IFDEF UNICODE}
  if srcRec.IsEmpty then Exit;
  dstRec := Rect(x, y, x + srcRec.Width, y + srcRec.Height);
{$ELSE}
  if IsRectEmpty(srcRec) then Exit;
  dstRec :=
    Rect(x, y, x + srcRec.Right - srcRec.Left, y + srcRec.Bottom - srcRec.Top);
{$ENDIF}
  if dstRec.Right > Width then
  begin
    dec(dstRec.Right, dstRec.Right - Width);
    dec(srcRec.Right, dstRec.Right - Width);
  end;
  if dstRec.Bottom > Height then
  begin
    dec(dstRec.Bottom, dstRec.Bottom - Height);
    dec(srcRec.Bottom, dstRec.Bottom - Height);
  end;
{$IFDEF UNICODE}
  if srcRec.IsEmpty then Exit;
{$ELSE}
  if IsRectEmpty(srcRec) then Exit;
{$ENDIF}

  j := 0;
  for i := Height-dstRec.Bottom to Height-dstRec.Top-1 do
  begin
    c := @Pixels[i * Width + dstRec.Left];
    move(image.Pixels[(srcRec.Top + j) * image.Width + srcRec.Left],
      c^, (dstRec.Right - dstRec.Left) * sizeof(TColor32));
    inc(j);
  end;
end;
//------------------------------------------------------------------------------

procedure TBitmap32.Crop(rec: TRect);
var
  i,w,h: integer;
  c: PColor32Array;
  newPixels: TColor32Array;
begin
  if rec.Left < 0 then rec.Left := 0;
  if rec.Right > Width then rec.Right := Width;
  if rec.Top < 0 then rec.Top := 0;
  if rec.Bottom > Height then rec.Bottom := Height;
  if (rec.Right > rec.Left) and (rec.Bottom > rec.Top) then
  begin
    w := rec.Right-rec.Left;
    h := rec.Bottom-rec.Top;
    setLength(newPixels, w * h);
    for i := Height-rec.Bottom to Height-rec.Top-1 do
    begin
      c := @Pixels[i * Width + rec.Left];
      move(c^, newPixels[(i-Height+rec.Bottom) * w], w * sizeof(TColor32));
    end;
    Resize(w,h);
    fPixels := newPixels;
  end else
    Resize(0,0);
end;
//------------------------------------------------------------------------------

procedure TBitmap32.CropTransparentPixels;
var
  x,y, x1,x2,y1,y2: integer;
  rec: TRect;
  found: Boolean;
begin
  y1 := 0; y2 := 0;
  found := false;
  for y := 0 to Height -1 do
  begin
    for x := 0 to Width -1 do
      if TARGB(fPixels[y * Width + x]).A > 0 then
      begin
        y1 := y;
        found := true;
        break;
      end;
    if found then break;
  end;

  if not found then
  begin
    SetSize(0, 0);
    Exit;
  end;

  found := false;
  for y := Height -1 downto 0 do
  begin
    for x := 0 to Width -1 do
      if TARGB(fPixels[y * Width + x]).A > 0 then
      begin
        y2 := y;
        found := true;
        break;
      end;
    if found then break;
  end;

  x1 := Width; x2 := 0;
  for y := y1 to y2 do
    for x := 0 to Width -1 do
      if TARGB(fPixels[y * Width + x]).A > 0 then
      begin
        if x < x1 then x1 := x;
        if x > x2 then x2 := x;
      end;

  rec := Rect(x1, height-y2-1, x2+1, height-y1);
  Crop(rec);
end;
//------------------------------------------------------------------------------

procedure TBitmap32.AssignTo(dst: TBitmap32);
begin
  dst.SetSize(width, height, false);
  if (width > 0) and (height > 0) then
    move(fPixels[0], dst.fPixels[0], width * height * sizeof(TColor32));
end;
//------------------------------------------------------------------------------

procedure TBitmap32.SetSize(width, height: integer; zeroFill: Boolean);
begin
  fwidth := width;
  fheight := height;
  setLength(fPixels, width * height);
  if zeroFill and (width > 0) and (height > 0) then
    FillChar(fPixels[0], width * height * Sizeof(TColor32), 0);
end;
//------------------------------------------------------------------------------

procedure TBitmap32.Resize(width, height: integer);
begin
  if (width <= 0) or (height <= 0) then
  begin
    fwidth := 0;
    fheight := 0;
    fPixels := nil;
  end
  else if (width <> fwidth) or (height <> fheight) then
  begin
    if fAntiAliase then
      fPixels := DoResizeAA(width, height) else
      fPixels := DoResize(width, height);
    fwidth := width;
    fheight := height;
  end;
end;
//------------------------------------------------------------------------------

function TBitmap32.DoResize(newWidth, newHeight: integer): TColor32Array;
var
  x, y, srcX, srcY: integer;
  scaledXi, scaledYi: TArrayOfInteger;
begin
  SetLength(result, newWidth * newHeight * sizeof(TColor32));
  if (fWidth = 0) or (fHeight = 0) or (newWidth = 0) or (newHeight = 0) then
    Exit
  else if ((fWidth = newWidth) and (fHeight = newHeight)) then
  begin
    Move(fPixels[0], result[0], newWidth * newHeight * sizeof(TColor32));
    Exit;
  end;

  SetLength(scaledXi, newWidth + 1);
  for x := 0 to newWidth do
    scaledXi[x] := Floor(x / newWidth * fWidth);
  SetLength(scaledYi, newHeight + 1);
  for y := 0 to newHeight do
    scaledYi[y] := Floor(y / newHeight * fHeight);

  for y := 0 to newHeight - 1 do
  begin
    srcY := scaledYi[y];
    if (srcY < 0) or (srcY >= fHeight) then Continue;
    for x := 0 to newWidth - 1 do
    begin
      srcX := scaledXi[x];
      if (srcX >= 0) and (srcX < fWidth) then
        result[x + y * newWidth] := fPixels[srcX + srcY * fWidth];
    end;
  end;
end;
//------------------------------------------------------------------------------

function TBitmap32.DoResizeAA(newWidth, newHeight: integer): TColor32Array;
var
  x,y, x256,y256,xx256,yy256: integer;
  xx,yy, sx,sy: double;
begin
  SetLength(result, newWidth * newHeight);

  if (fWidth = 0) or (fHeight = 0) or
    (newWidth = 0) or (newHeight = 0) then Exit

  else if ((fWidth = newWidth) and (fHeight = newHeight)) then
  begin
    Move(fPixels[0], result[0], newWidth * newHeight * sizeof(TColor32));
    Exit;
  end;

  sx := fWidth/newWidth;
  sy := fHeight/newHeight;
  yy := 0; y256 := 0;
  for y := 0 to newHeight - 1 do
  begin
    xx := 0; x256 := 0;
    yy := yy + sy;
    yy256 := Round(yy * 256);
    for x := 0 to newWidth - 1 do
    begin
      xx := xx + sx;
      xx256 := Round(xx * 256);
      result[x + y * newWidth] :=
        GetWeightedColor(fPixels, x256, y256, xx256, yy256, fWidth);
      x256 := xx256;
    end;
    y256 := yy256;
  end;
end;
//------------------------------------------------------------------------------

procedure TBitmap32.Scale(value: single);
var
  w, h: integer;
begin
  w := Round(width * value);
  h := Round(height * value);
  ReSize(w, h);
end;
//------------------------------------------------------------------------------

function PointD(const X, Y: Double): TPointD;
begin
  Result.X := X;
  Result.Y := Y;
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

function RotateRectD(const rec: TRectD; angleRad: double): TRectD;
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

function TBitmap32.GetWeightedPixel(x256, y256: integer): TColor32;
var
  xi, yi, weight: integer;
  color: TWeightedColor;
  xf, yf: cardinal;
begin
  if (x256 < -$FF) or (y256 < -$FF) or
    (x256 >= fWidth * $100) or (y256 >= fHeight * $100) then
  begin
    result := clNone32;
    Exit;
  end;
  xi := abs(x256) shr 8;
  xf := x256 and $FF;
  yi := abs(y256) shr 8;
  yf := y256 and $FF;

  color.Reset;
  weight := (($100 - xf) * ($100 - yf)) shr 8;         //top-left
  if (x256 < 0) or (y256 < 0) then
    color.AddWeight(weight) else
    color.Add(fPixels[xi + yi * fWidth], weight);

  weight := (xf * ($100 - yf)) shr 8;                  //top-right
  if (xi + 1 >= fWidth) or (y256 < 0) then
    color.AddWeight(weight) else
    color.Add(fPixels[xi + 1 + yi * fWidth], weight);

  weight := (($100 - xf) * yf) shr 8;                  //bottom-left
  if (x256 < 0) or (yi + 1 = fHeight) then
    color.AddWeight(weight) else
    color.Add(fPixels[xi + (yi +1) * fWidth], weight);

  weight := (xf * yf) shr 8;                           //bottom-right
  if (xi + 1 >= fWidth) or (yi + 1 = fHeight) then
    color.AddWeight(weight) else
    color.Add(fPixels[(xi + 1)  + (yi + 1) * fWidth], weight);
  Result := color.Color;
end;
//------------------------------------------------------------------------------

procedure TBitmap32.Rotate(angleRads: single);
var
  tmp: TColor32Array;
  x, y, xi, yi, newWidth, newHeight: integer;
  sinA, cosA, dx, dy: double;
  pt, cp, cp2: TPointD;
  rec: TRectD;
  dstColor: PColor32;
begin
  if IsEmpty then Exit;
  sinA := Sin(-angleRads); cosA := cos(-angleRads);
  cp := PointD(width / 2, height / 2);
  rec.Left := 0; rec.Top := 0;
  rec.Right := Width; rec.Bottom := Height;
  rec := RotateRectD(rec, angleRads);
  newWidth := Ceil(rec.Right - rec.Left);
  newHeight := Ceil(rec.Bottom - rec.Top);
  cp2 := PointD(newWidth / 2, newHeight / 2);
  SetLength(tmp, newWidth * newHeight);
  dstColor := @tmp[0];
  dx := (newWidth - fWidth) / 2;
  dy := (newHeight - fHeight) / 2;
  if EnableAntiAliase then
  begin
    for y := 0 to newHeight -1 do
      for x := 0 to newWidth -1 do
      begin
        pt := PointD(x, y);
        RotatePt(pt, cp2, sinA, cosA);
        xi := Round((pt.X - dx) * 256);
        yi := Round((pt.Y - dy) * 256);
        dstColor^ := GetWeightedPixel(xi, yi);
        inc(dstColor);
      end;
  end else
  begin
    for y := 0 to newHeight -1 do
      for x := 0 to newWidth -1 do
      begin
        pt := PointD(x, y);
        RotatePt(pt, cp2, sinA, cosA);
        xi := Round(pt.X - dx);
        yi := Round(pt.Y - dy);
        if (xi < 0) or (xi >= Width) or (yi < 0) or (yi >= Height) then
          dstColor^ := clNone32
        else
          dstColor^ := fPixels[xi + yi * Width];
        inc(dstColor);
      end;
  end;
  fPixels := tmp;
  fWidth := newWidth;
  fHeight := newHeight;
end;
//------------------------------------------------------------------------------

procedure TBitmap32.FlipVertical;
var
  i: integer;
  a: TColor32Array;
  row: PColor32;
begin
  if IsEmpty then Exit;
  SetLength(a, fWidth * fHeight);
  for i := 0 to fHeight -1 do
  begin
    row := GetPixelRow(i);
    move(row^, a[i * fWidth], fWidth * sizeof(TColor32));
  end;
  fPixels := a;
end;
//------------------------------------------------------------------------------

procedure TBitmap32.FlipHorizontal;
var
  i,j, widthLess1: integer;
  a: TColor32Array;
  row: PColor32;
begin
  if IsEmpty then Exit;
  SetLength(a, fWidth);
  widthLess1 := fWidth -1;
  for i := 0 to fHeight -1 do
  begin
    row := GetPixelRow(i);
    move(row^, a[0], fWidth * sizeof(TColor32));
    for j := 0 to widthLess1 do
    begin
      row^ := a[widthLess1 - j];
      inc(row);
    end;
  end;
end;
//------------------------------------------------------------------------------

function TBitmap32.CountColors: integer;
var
  allColors: TArrayOfByte;
  i,j: integer;
  c: PColor32;
const
  two24 = 256 * 256 * 256;
begin
  result := 0;
  if IsEmpty then Exit;
  SetLength(allColors, two24);
  fillChar(allColors[0], two24, 0);
  c := PixelBase;
  for i := 0 to Width * Height -1 do
  begin
    with PARGB(c)^ do j := (R shl 16) + (G shl 8) + B;
    allColors[j] := 1;
    inc(c);
  end;
  for i := 0 to two24 -1 do
    if allColors[i] = 1 then inc(Result);
end;
//------------------------------------------------------------------------------

function TBitmap32.SaveToFile(const filename: string): Boolean;
var
  idx: integer;
  ext: string;
  bitmap32ExtClass: TBitmap32ExtClass;
begin
  result := false;
  ext := Lowercase(ExtractFileExt(filename));
  if ext = '' then Exit;
  if ext[1] = '.' then Delete(ext, 1,1);
  if not Bitmap32ExtClassList.Find(ext, idx) then Exit;
  bitmap32ExtClass := TBitmap32ExtClass(Bitmap32ExtClassList.Objects[idx]);
  result := bitmap32ExtClass.SaveToFile(filename, self);
end;
//------------------------------------------------------------------------------

function TBitmap32.LoadFromFile(const filename: string): Boolean;
var
  idx: integer;
  ext: string;
  bitmap32ExtClass: TBitmap32ExtClass;
begin
  result := false;
  ext := Lowercase(ExtractFileExt(filename));
  if ext = '' then Exit;
  if ext[1] = '.' then Delete(ext, 1,1);
  if not Bitmap32ExtClassList.Find(ext, idx) then Exit;
  bitmap32ExtClass := TBitmap32ExtClass(Bitmap32ExtClassList.Objects[idx]);
  result := bitmap32ExtClass.LoadFromFile(filename, self);
end;
//------------------------------------------------------------------------------

function TBitmap32.GetPixel(x, y: integer): TColor32;
begin
  result := fPixels[y * width + x];
end;
//------------------------------------------------------------------------------

function TBitmap32.GetPixelRow(index: integer): PColor32;
begin
  //assumes inverted image and note no range checking
  result := @fPixels[(height-index-1) * width];
end;
//------------------------------------------------------------------------------

procedure TBitmap32.SetPixel(x,y: integer; color: TColor32);
begin
  fPixels[y * width + x] := color;
end;
//------------------------------------------------------------------------------

function TBitmap32.GetIsEmpty: Boolean;
begin
  result := fPixels = nil;
end;
//------------------------------------------------------------------------------

function TBitmap32.GetPixelBase: PColor32;
begin
  if IsEmpty then result := nil
  else result := @fPixels[0];
end;
//------------------------------------------------------------------------------

procedure TBitmap32.DrawTo(dst: TBitmap32);
begin
  if (dst = self) then Exit;

  //if bmDest is empty then assume a simple copy ...
  if dst.IsEmpty then dst.SetSize(width, height, false);
  if (dst.Width = Width) and (dst.Height = Height) then
  begin
    //do simple copy...
    if not IsEmpty then
      Move(fPixels[0], dst.fPixels[0], Length(fPixels) * sizeof(TColor32));
  end
  else if fAntiAliase then
    dst.fPixels := DoResizeAA(dst.Width, dst.Height)
  else
    dst.fPixels := DoResize(dst.Width, dst.Height);
end;

//------------------------------------------------------------------------------
// TWeightedColor record
//------------------------------------------------------------------------------

procedure TWeightedColor.Reset;
begin
  fAddCount := 0;
  fAlphaTot := 0;
  fColorTotR := 0;
  fColorTotG := 0;
  fColorTotB := 0;
end;
//------------------------------------------------------------------------------

procedure TWeightedColor.AddWeight(weight: cardinal);
begin
  inc(fAddCount, weight);
end;
//------------------------------------------------------------------------------

procedure TWeightedColor.Add(c: TColor32; weight: cardinal);
var
  a: cardinal;
  argb: TARGB absolute c;
begin
  inc(fAddCount, weight);
  a := weight * argb.A;
  if a = 0 then Exit;
  inc(fAlphaTot, a);
  inc(fColorTotB, (a * argb.B));
  inc(fColorTotG, (a * argb.G));
  inc(fColorTotR, (a * argb.R));
end;
//------------------------------------------------------------------------------

function DivRound(num, denom: Cardinal): Cardinal;
  {$IFDEF INLINING} inline; {$ENDIF}
begin
  result := (num  + denom div 2) div denom;
end;
//------------------------------------------------------------------------------

function TWeightedColor.GetColor: TColor32;
var
  a: byte;
  halfAlphaTot: cardinal;
  argb: TARGB absolute result;
begin
  result := clNone32;
  if (fAlphaTot = 0) then Exit;
  a := DivRound(fAlphaTot, fAddCount);
  if (a = 0) then Exit;
  argb.A := a;
  halfAlphaTot := fAlphaTot div 2;
  //nb: alpha weighting is applied to colors when added
  //so we now need to div by fAlphaTot (with rounding) here ...
  argb.R := (fColorTotR + halfAlphaTot) div fAlphaTot;
  argb.G := (fColorTotG + halfAlphaTot) div fAlphaTot;
  argb.B := (fColorTotB + halfAlphaTot) div fAlphaTot;
end;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

initialization
  Bitmap32ExtClassList := TStringList.Create;
finalization
  Bitmap32ExtClassList.Free;

end.
