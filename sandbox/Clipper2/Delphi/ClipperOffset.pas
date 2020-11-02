unit ClipperOffset;

(*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Version   :  10.0 (beta)                                                     *
* Date      :  24 October 2020                                                 *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2017                                         *
* Purpose   :  Offset paths and clipping solutions                             *
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

interface

uses
  SysUtils, Classes, Math, ClipperCore, Clipper;

type

  TJoinType = (jtSquare, jtRound, jtMiter);
  TEndType = (etPolygon, etOpenJoined, etOpenButt, etOpenSquare, etOpenRound);

  TPathGroup = class
	  paths     : TPathsD;
	  joinType  : TJoinType;
	  endType   : TEndType;
    constructor Create(const paths: TPathsD; jt: TJoinType; et: TEndType);
  end;

  TClipperOffset = class
  private
    fDelta       : Double;
    fJoinType    : TJoinType;
    fTmpLimit    : Double;
    fMiterLimit  : Double;
    fStepsPerRad : Double;
    fNorms       : TPathD;
    fInGroups    : TList;
    fInPath      : TPathD;
    fOutPath     : TPathD;
    fOutPathLen  : Integer;
    fSolution    : TPathsD;
    fArcTolerance: Double;
    fSinA        : Double;
    fCosA        : Double;

    procedure AddPoint(x,y: double); overload;
    procedure AddPoint(const pt: TPointD); overload;
    procedure DoSquare(j, k: Integer);
    procedure DoMiter(j, k: Integer; cosAplus1: Double);
    procedure DoRound(j, k: integer; isEnd: boolean);
    procedure OffsetPoint(j: Integer; var k: integer);

    procedure CheckPaths(et: TEndType);
    procedure BuildNormals;
    procedure DoOffset(const paths: TPathsD;
      delta: double; joinType: TJoinType; endType: TEndType);
    procedure OffsetPolygon;
    procedure OffsetOpenPath(endType: TEndType);
  public
    constructor Create(MiterLimit: double = 2.0; ArcTolerance: double = 0.0);
    destructor Destroy; override;
    procedure AddPath(const path: TPathD;
      joinType: TJoinType; endType: TEndType);
    procedure AddPaths(const paths: TPathsD;
      joinType: TJoinType; endType: TEndType);
    procedure Clear;
    procedure Execute(delta: Double; out solution: TPathsD);
    property MiterLimit: Double read fMiterLimit write fMiterLimit;
    property ArcTolerance: Double read fArcTolerance write fArcTolerance;
  end;

  function ClipperOffsetPaths(const paths: TPaths;
    delta: Double; jt: TJoinType; et: TEndType;
    miterLimit: double = 2.0): TPaths; overload;

  function ClipperOffsetPaths(const paths: TPathsD;
    delta: Double; jt: TJoinType; et: TEndType;
    miterLimit: double = 2.0): TPathsD; overload;

implementation

//OVERFLOWCHECKS is OFF as a necessary workaround for a Delphi compiler bug that
//very occasionally reports overflow errors while still returning correct values
//eg var A, B: Int64; begin A := -$13456780; B := -$73456789; A := A * B; end;
//see https://forums.embarcadero.com/message.jspa?messageID=871444
//nb: this issue has now been resolved in Delphi 10.2
{$OVERFLOWCHECKS OFF}

resourcestring
  rsClipperOffset = 'ClipperOffset error';

const
  Tolerance           : Double = 1.0E-15;
  DefaultArcFrac      : Double = 0.02;
  Two_Pi              : Double = 2 * PI;
  Quarter_Pi          : Double = 0.25 * PI;
  LowestIp            : TPoint64 = (X: High(Int64); Y: High(Int64));

//------------------------------------------------------------------------------
//  Miscellaneous offset support functions
//------------------------------------------------------------------------------

function PointD(const X, Y: Double): TPointD; overload;
begin
  Result.X := X;
  Result.Y := Y;
end;
//------------------------------------------------------------------------------

function PointD(const pt: TPoint64): TPointD; overload;
begin
  Result.X := pt.X;
  Result.Y := pt.Y;
end;
//------------------------------------------------------------------------------

function DistanceSqr(const pt1, pt2: TPointD): double;
begin
  Result := (pt1.X - pt2.X)*(pt1.X - pt2.X) + (pt1.Y - pt2.Y)*(pt1.Y - pt2.Y);
end;
//------------------------------------------------------------------------------

function GetUnitNormal(const pt1, pt2: TPointD): TPointD;
var
  dx, dy, inverseHypot: Double;
begin
  if (pt2.X = pt1.X) and (pt2.Y = pt1.Y) then
  begin
    Result.X := 0;
    Result.Y := 0;
    Exit;
  end;

  dx := (pt2.X - pt1.X);
  dy := (pt2.Y - pt1.Y);
  inverseHypot := 1 / Hypot(dx, dy);
  dx := dx * inverseHypot;
  dy := dy * inverseHypot;
  Result.X := dy;
  Result.Y := -dx
end;
//------------------------------------------------------------------------------

function GetLowestPolygonIdx(const paths: TPathsD): integer;
var
  i,j: integer;
  lp: TPointD;
  p: TPathD;
begin
	Result := -1;
	for i := 0 to High(paths) do
		if Length(paths[i]) > 0 then
    begin
			Result := i;
			lp := paths[i][0];
			break;
		end;
	if (Result < 0) then Exit;

	for i := Result to High(paths) do
	begin
		p := paths[i];
		for j := 0 to High(p) do
			if (p[j].Y > lp.Y) or ((p[j].Y = lp.Y) and (p[j].X < lp.X)) then
      begin
				Result := i;
				lp := p[j];
			end;
  end;
end;
//------------------------------------------------------------------------------

//function TurnsLeft(const a, b, c: TPointD): boolean;
//var
//  ab,bc: TPointD;
//begin
//  ab.X := b.X - a.X; ab.Y := b.Y - a.Y;
//  bc.X := b.X - c.X; bc.Y := b.Y - c.Y;
//  result := (ab.x * bc.y - ab.y * bc.x) > 0; //cross product
//end;
//---------------------------------------------------------------------------

function CopyPaths(const paths: TPathsD): TPathsD;
var
  i, len: integer;
begin
  len := Length(paths);
  SetLength(Result, len);
  for i := 0 to len -1 do
    Result[i] := Copy(paths[i], 0, Length(paths[i]));
end;

//------------------------------------------------------------------------------
// TPathGroup methods
//------------------------------------------------------------------------------

constructor TPathGroup.Create(const paths: TPathsD; jt: TJoinType; et: TEndType);
begin
  Self.paths := CopyPaths(paths);
  Self.joinType := jt;
  Self.endType := et;
end;

//------------------------------------------------------------------------------
// TClipperOffset methods
//------------------------------------------------------------------------------

constructor TClipperOffset.Create(MiterLimit: double; ArcTolerance: double);
begin
  inherited Create;
  fMiterLimit := MiterLimit;
  fArcTolerance := ArcTolerance;
  fInGroups     := TList.Create;
end;
//------------------------------------------------------------------------------

destructor TClipperOffset.Destroy;
begin
  Clear;
  fInGroups.Free;
  inherited;
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.Clear;
var
  i: integer;
begin
  for i := 0 to fInGroups.Count -1 do
    TPathGroup(fInGroups[i]).Free;
  fInGroups.Clear;
  fSolution := nil;
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.AddPath(const path: TPathD;
  joinType: TJoinType; endType: TEndType);
var
  paths: TPathsD;
begin
  if not assigned(path) then Exit;
  SetLength(paths, 1);
  paths[0] := path;
  AddPaths(Paths, joinType, endType);
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.AddPaths(const paths: TPathsD;
  joinType: TJoinType; endType: TEndType);
var
  node: TPathGroup;
begin
  if Length(paths) = 0 then Exit;
  node := TPathGroup.Create(paths, joinType, endType);
  fInGroups.Add(node);
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.CheckPaths(et: TEndType);
var
  i,j, len, minLen: Integer;
  openPaths: Boolean;
begin
  for i := 0 to fInGroups.Count -1 do
    with TPathGroup(fInGroups[i]) do
    begin
      openPaths := not (et in [etPolygon, etOpenJoined]);
      if openPaths then minLen := 1 else minLen := 3;
      for j := 0 to High(paths) do
      begin
        StripDuplicates(paths[j]);
        len := length(paths[j]);
        if (len > 1) and not openPaths and
          PointsEqual(paths[j][0], paths[j][len-1]) then
        begin
          setlength(paths[j], len -1);
          dec(len);
        end;
        if len < minLen then paths[j] := nil;
      end;
    end;
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.DoOffset(const paths: TPathsD;
  delta: double; joinType: TJoinType; endType: TEndType);
var
  i, lowestIdx: Integer;
  arcTol, steps: Double;
  tmpEndType: TEndType;
  outPaths: TPathsD;
  outPathsI: TPaths;
  openPaths, isClockwise: Boolean;
begin

  fJoinType := joinType;
  CheckPaths(endType);

  openPaths := not (endType in [etPolygon, etOpenJoined]);
  if not openPaths then
  begin
    //the lowermost polygon must be an outer polygon. So we can use that as the
    //designated orientation for outer polygons (needed for tidy-up clipping)
    lowestIdx := GetLowestPolygonIdx(paths);
    if lowestIdx < 0 then Exit;
    isClockwise := Area(paths[lowestIdx]) > 0;
    if not isClockwise then delta := -delta;
    fDelta := delta;
  end else
  begin
    fDelta := Abs(delta) /2;
    isClockwise := true;
  end;

  //FMiterLimit: see offset_triginometry3.svg
  if fMiterLimit > 1 then
    fTmpLimit := 2/(sqr(fMiterLimit)) else
    fTmpLimit := 2;

  if (fArcTolerance <= DefaultArcFrac) then
    arcTol := DefaultArcFrac else
    arcTol := fArcTolerance;

  //calculate a sensible number of steps (for 360 deg for the given offset
  if (joinType = jtRound) or (endType = etOpenRound) then
  begin
    //get steps per 360 degrees (see offset_triginometry2.svg)
    steps := PI / ArcCos(1 - arcTol / Abs(fDelta));
    steps := Min(steps, Max(6, Abs(fDelta) * Pi)); //avoids excessive precision
    fStepsPerRad := steps / Two_Pi;
  end;

  outPaths := nil;
  for i := 0 to High(paths) do
  begin
    fInPath := paths[i];
    if fInPath = nil then Continue;

    fNorms := nil;
    fOutPath := nil;
    fOutPathLen := 0;

    if Length(fInPath) = 1 then
    begin
      //construct a circular or square 'point' using OffsetOpenPath
      tmpEndType := endType;
      try
        if endType = etOpenButt then endType := etOpenSquare;
        SetLength(fInPath, 2);
        fInPath[1] := fInPath[0];
        SetLength(fNorms, 2);
        fNorms[0] := PointD(1,0);
        OffsetOpenPath(endType);
      finally
        endType := tmpEndType;
      end;
    end else
    begin
      BuildNormals;

      if endType = etPolygon then
        OffsetPolygon
      else if endType = etOpenJoined then
      begin
        fInPath := Copy(paths[i], 0, Length(paths[i]));
        OffsetPolygon;
        SetLength(fOutPath, fOutPathLen);
        AppendPath(outPaths, fOutPath);
        fOutPath := nil;
        fOutPathLen := 0;
        fInPath := ReversePath(fInPath);
        BuildNormals;
        OffsetPolygon;
      end else
        OffsetOpenPath(endType);
    end;

    if fOutPathLen = 0 then Continue;
    SetLength(fOutPath, fOutPathLen);
    AppendPath(outPaths, fOutPath);
  end;

  outPathsI := ScalePaths(outPaths, 100, 100);
  //clean up self-intersections ...
  with TClipper.Create do
  try
    AddPaths(outPathsI, ptSubject);
    if not isClockwise then
      Execute(ctUnion, frNegative, outPathsI) else
      Execute(ctUnion, frPositive, outPathsI);
  finally
    free;
  end;
  outPaths := ScalePathsD(outPathsI, 0.01, 0.01);

  AppendPaths(fSolution, outPaths);
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.BuildNormals;
var
  i, len: integer;
begin
  len := Length(fInPath);
  SetLength(fNorms, len);
  for i := 0 to len-2 do
    fNorms[i] := GetUnitNormal(fInPath[i], fInPath[i+1]);
  fNorms[len -1] := GetUnitNormal(fInPath[len -1], fInPath[0]);
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.OffsetPolygon;
var
  i,j: integer;
begin
  j := high(fInPath);
  for i := 0 to high(fInPath) do
    OffsetPoint(i, j);
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.OffsetOpenPath(endType: TEndType);

  procedure DoButtEnd(highI: integer);
  begin
    AddPoint(fInPath[highI].X + fNorms[highI-1].X *fDelta,
      fInPath[highI].Y + fNorms[highI-1].Y * fDelta);
    AddPoint(fInPath[highI].X - fNorms[highI-1].X *fDelta,
      fInPath[highI].Y - fNorms[highI-1].Y * fDelta);
  end;

  procedure DoButtStart;
  begin
    AddPoint(fInPath[0].X + fNorms[1].X *fDelta,
      fInPath[0].Y + fNorms[1].Y * fDelta);
    AddPoint(fInPath[0].X - fNorms[1].X *fDelta,
      fInPath[0].Y - fNorms[1].Y * fDelta);
  end;

var
  i, k, highI: integer;
begin
  highI := high(fInPath);
  k := 0;
  for i := 1 to highI -1 do
    OffsetPoint(i, k);

  k := highI -1;
  fNorms[highI].X := -fNorms[k].X;
  fNorms[highI].Y := -fNorms[k].Y;

 //cap the end first ...
  case endType of
    etOpenButt: DoButtEnd(highI);
    etOpenRound: DoRound(highI, k, true);
    else DoSquare(highI, k);
  end;

  //reverse normals ...
  for i := highI -1 downto 1 do
  begin
    fNorms[i].X := -fNorms[i-1].X;
    fNorms[i].Y := -fNorms[i-1].Y;
  end;
  fNorms[0].X := -fNorms[1].X;
  fNorms[0].Y := -fNorms[1].Y;

  k := highI;
  for i := highI -1 downto 1 do
    OffsetPoint(i, k);

  //now cap the start ...
  case endType of
    etOpenButt: DoButtStart;
    etOpenRound: DoRound(0, 1, true);
    else doSquare(0, 1);
  end;
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.Execute(delta: Double; out solution: TPathsD);
var
  i: integer;
begin
  solution := nil;
  fSolution := nil;
  if fInGroups.Count = 0 then Exit;

  //if a Zero offset, then simply copy paths to FSolution and Exit
  if abs(delta) < Tolerance then
  begin
    for i := 0 to fInGroups.Count -1 do
      with TPathGroup(fInGroups[i]) do
          AppendPaths(solution, paths);
    Exit;
  end;

  for i := 0 to fInGroups.Count -1 do
    with TPathGroup(fInGroups[i]) do
      DoOffset(paths, delta, jointype, endtype);

  solution := fSolution;
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.AddPoint(x,y: double);
const
  BuffLength = 32;
var
  pt: TPointD;
begin
  pt := PointD(x,y);
  if fOutPathLen = length(fOutPath) then
    SetLength(fOutPath, fOutPathLen + BuffLength);
  if (fOutPathLen > 0) and PointsEqual(fOutPath[fOutPathLen-1], pt) then Exit;
  fOutPath[fOutPathLen] := pt;
  Inc(fOutPathLen);
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.AddPoint(const pt: TPointD);
const
  BuffLength = 32;
begin
  if fOutPathLen = length(fOutPath) then
    SetLength(fOutPath, fOutPathLen + BuffLength);
  if (fOutPathLen > 0) and PointsEqual(fOutPath[fOutPathLen-1], pt) then Exit;
  fOutPath[fOutPathLen] := pt;
  Inc(fOutPathLen);
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.DoSquare(j, k: Integer);
begin
  //Two vertices, one using the prior offset's (k) normal one the current (j).
  //Do a 'normal' offset (by delta) and then another by 'de-normaling' the
  //normal hence parallel to the direction of the respective edges.
  if (fDelta > 0) then
  begin
    AddPoint(
      fInPath[j].X + fDelta * (fNorms[k].X - fNorms[k].Y),
      fInPath[j].Y + fDelta * (fNorms[k].Y + fNorms[k].X));

    AddPoint(
      fInPath[j].X + fDelta * (fNorms[j].X + fNorms[j].Y),
      fInPath[j].Y + fDelta * (fNorms[j].Y - fNorms[j].X));
  end else
  begin
    AddPoint(
      fInPath[j].X + fDelta * (fNorms[k].X + fNorms[k].Y),
      fInPath[j].Y + fDelta * (fNorms[k].Y - fNorms[k].X));
    AddPoint(
      fInPath[j].X + fDelta * (fNorms[j].X - fNorms[j].Y),
      fInPath[j].Y + fDelta * (fNorms[j].Y + fNorms[j].X));
  end;
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.DoMiter(j, k: Integer; cosAplus1: Double);
var
  q: Double;
begin
  //see offset_triginometry4.svg
  q := fDelta / cosAplus1;
  AddPoint(fInPath[j].X + (fNorms[k].X + fNorms[j].X)*q,
    fInPath[j].Y + (fNorms[k].Y + fNorms[j].Y)*q);
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.DoRound(j, k: Integer; isEnd: boolean);
var
  i, steps: Integer;
  stepSin, stepCos: Extended;
  a, absA: Double;
  pt, pt2: TPointD;
begin
  //a convex vertex ...

  pt := fInPath[j];
  pt2.X := fNorms[k].X * fDelta;
  pt2.Y := fNorms[k].Y * fDelta;
  AddPoint(pt.X + pt2.X, pt.Y + pt2.Y);

  if isEnd then a := PI
  else a := ArcTan2(fSinA, fCosA);

  absA := abs(a);
	if (absA < quarter_pi) then
		steps := Round(fStepsPerRad * absA) else
		steps := Max(3, Round(fStepsPerRad * absA));

  if steps > 0 then
  begin
    Math.SinCos(a / steps, stepSin, stepCos);
    for i := 1 to steps -1 do
    begin
      pt2 := PointD(pt2.X * stepCos - stepSin * pt2.Y,
        pt2.X * stepSin + pt2.Y * stepCos);
      AddPoint(pt.X + pt2.X, pt.Y + pt2.Y);
    end;
  end;

  pt := fInPath[j];
  pt2.X := fNorms[j].X * fDelta;
  pt2.Y := fNorms[j].Y * fDelta;
  AddPoint(pt.X + pt2.X, pt.Y + pt2.Y);
end;
//------------------------------------------------------------------------------

procedure TClipperOffset.OffsetPoint(j: Integer; var k: integer);
begin
  //A: angle between adjoining edges (on left side WRT winding direction).
  //A == 0 deg (or A == 360 deg): collinear edges heading in same direction
  //A == 180 deg: collinear edges heading in opposite directions (ie a 'spike')
  //sin(A) < 0: convex on left.
  //cos(A) > 0: angles on both left and right sides > 90 degrees
  fSinA := (fNorms[k].X * fNorms[j].Y - fNorms[j].X * fNorms[k].Y);

  if (fSinA < 0.01) and (fSinA > -0.01) then
  begin
    k := j;
    Exit;
  end;

  if (fSinA > 1.0) then fSinA := 1.0
  else if (fSinA < -1.0) then fSinA := -1.0;

  if fSinA * fDelta < 0 then //ie a concave offset
  begin
    AddPoint(fInPath[j].X + fNorms[k].X * fDelta,
      fInPath[j].Y + fNorms[k].Y * fDelta);
    AddPoint(fInPath[j]); //this aids with clipping removal later
    AddPoint(fInPath[j].X + fNorms[j].X * fDelta,
      fInPath[j].Y + fNorms[j].Y * fDelta);
  end else
  begin
    //convex offsets here ...
    case fJoinType of
      jtMiter:
        //see offset_triginometry3.svg
        if (1 + fCosA < fTmpLimit) then DoSquare(j, k)
        else DoMiter(j, k, 1 + fCosA);
      jtSquare:
        begin
          fCosA := (fNorms[j].X * fNorms[k].X + fNorms[j].Y * fNorms[k].Y);
          //angles >= 90 deg. don't need squaring
          if fCosA >= 0 then
            DoMiter(j, k, 1 + fCosA) else
            DoSquare(j, k);
        end
      else
      begin
        fCosA := (fNorms[j].X * fNorms[k].X + fNorms[j].Y * fNorms[k].Y);
        DoRound(j, k, false);
      end;
    end;
  end;
  k := j;
end;
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

function ClipperOffsetPaths(const paths: TPaths;
  delta: Double; jt: TJoinType; et: TEndType; miterLimit: double): TPaths;
var
  pp: TPathsD;
begin
  pp := ScalePathsD(paths, 100, 100);
  with TClipperOffset.Create(miterLimit) do
  try
    AddPaths(pp, jt, et);
    Execute(delta, pp);
  finally
    free;
  end;
  Result := ScalePaths(pp, 0.01, 0.01);
end;
//------------------------------------------------------------------------------

function ClipperOffsetPaths(const paths: TPathsD;
  delta: Double; jt: TJoinType; et: TEndType; miterLimit: double): TPathsD;
begin
  with TClipperOffset.Create(miterLimit) do
  try
    AddPaths(paths, jt, et);
    Execute(delta, Result);
  finally
    free;
  end;
end;
//------------------------------------------------------------------------------

end.
