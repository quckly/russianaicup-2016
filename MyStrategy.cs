using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading;

#if QDEBUG
using System.Net.Sockets;
using System.IO;
using System.Text;
using System.Globalization;
#endif

using Com.CodeGame.CodeWizards2016.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeWizards2016.DevKit.CSharpCgdk {
    public sealed class MyStrategy : IStrategy {

        // Global variables
        private bool STOP = false;
        private VectorD NextPoint = new VectorD(500, 800);

        // For one move variables
        Wizard self; World world; Game game; Move move;
        private Physic physic;
        private List<Task> moveTasks;

        public void CalculateNextMovement()
        {
            RandomRun();
        }

        public void RandomRun()
        {
            var path = physic.FindPathTo(NextPoint);

            if (path == null || path.PathLength <= 5)
            {
                NextPoint = NextPoint == new VectorD(2000, 300) ? new VectorD(500, 800) : new VectorD(2000, 300);
                path = physic.FindPathTo(NextPoint);
            }

            var target = physic.GetPointFromCell(path.Reverse().ToArray()[1].LastStep.Vec);

            foreach (var ppp in path.Reverse())
            {
                var pp = physic.GetPointFromCell(ppp.LastStep.Vec);
                vc.FillCircle(pp.X, pp.Y, 4.0f, 0, 0, 1);
            }

            var task = new Task();

            if (!STOP)
            {
                SetRunToPoint(task.Movement, target);
            }
            task.Movement.Action = ActionType.Staff;

            moveTasks.Add(task);
        }

        private void SetRunToPoint(Move move, VectorD target)
        {
            var vectorToTarget = (target - new VectorD(self));

            if (vectorToTarget.SqLength < 4)
            {
                move.Speed = move.StrafeSpeed = 0;
                return;
            }

            double angle = self.GetAngleTo(target.X, target.Y);

            VectorD vect = QMath.AngleToVector(angle);

            double coof = new VectorD(vect.X / 4.0, vect.Y / 3.0).Length;
            vect *= 1.0 / coof;

            move.StrafeSpeed = vect.Y;
            move.Speed = vect.X;

            //vc.Line(self.X, self.Y, (self.X + vect.X), (self.Y + vect.Y), 0.7f, 0.9f, 0.1f);

            move.Turn = angle;
        }

        private void ApplyNextMovement()
        {
            var bestTask = moveTasks
                .OrderBy(task => task.Cost)
                .FirstOrDefault();

            if (bestTask == null) // This is not be true never
            {
                move.Action = ActionType.Staff;
                return;
            }

            move.CopyFrom(bestTask.Movement);
        }

        public void Move(Wizard self, World world, Game game, Move move)
        {
#if QDEBUG
            BeginDebug();
#endif

            InitMove(self, world, game, move);

            CalculateNextMovement();

            ApplyNextMovement();

#if QDEBUG
            EndDebug();
#endif
        }

        void InitMove(Wizard self, World world, Game game, Move move)
        {
            this.self = self;
            this.world = world;
            this.game = game;
            this.move = move;

            physic = new Physic(self,world,game,move);

            moveTasks = new List<Task>();

            // Dirty hacks
            AStar.SavedVectors = AStar.ToSaveVectors;
            AStar.ToSaveVectors = new Dictionary<Vector, Vector>();
        }

#if QDEBUG
        private static bool tryConnect = false;
        public static VisualClient vc = null;

        private void DrawCells()
        {
            for (int i = 0; i < physic.CellGridCount; i++)
            {
                for (int j = 0; j < physic.CellGridCount; j++)
                {
                    var coords = new VectorD(i * physic.CellGridWidth + physic.CellGridWidth / 2, j * physic.CellGridWidth + physic.CellGridWidth / 2);
                    var free = physic.cells[i, j];//physic.IsFreeCell(new Vector(i, j));

                    if (free == Physic.CellStatus.Free)
                        vc.FillCircle(coords.X, coords.Y, 2.0f, 0, 1, 0);
                    else if (free == Physic.CellStatus.Busy)
                        vc.FillCircle(coords.X, coords.Y, 2.0f, 1, 0, 0);
                }
            }
        }

        private void BeginDebug()
        {
            if (!tryConnect && vc == null)
            {
                tryConnect = true;

                try
                {
                    vc = new VisualClient("127.0.0.1", 13579);
                }
                catch { }
            }

            if (vc != null)
            {
                var culture = System.Threading.Thread.CurrentThread.CurrentCulture.Clone() as System.Globalization.CultureInfo;
                culture.NumberFormat.NumberDecimalSeparator = ".";
                System.Threading.Thread.CurrentThread.CurrentCulture = culture;

                vc.BeginPost();
            }
        }

        private void EndDebug()
        {
            // Draw smth
            DrawCells();

            vc.EndPost();
        }
#endif
    }

    public class Physic
    {
        Wizard self; World world; Game game; Move move;

        // Word model
        // Clusters
        public readonly int ClusterGridWidth = 100;
        public int ClusterGridCount { get; private set; }

        private List<CircularUnit>[,] clusters;

        public static readonly Vector[] neighboursVectors = {new Vector(-1, -1), new Vector(0, -1), new Vector(1, -1), new Vector(-1, 0), new Vector(1, 0), new Vector(-1, 1), new Vector(0, 1), new Vector(1,1) };
        public static readonly Vector[] neisAndMeClusters = neighboursVectors.Union(new[] {new Vector(0, 0)}).ToArray();

        private Building[] basesBuildings;

        // Cells
        // {1,2,4,5,8,10,16,20,25,32,40,50,80,100,125,160,200,250,400,500,800,1000,2000,4000}
        public readonly int CellGridWidth = 20;
        public int CellGridCount { get; private set; }

        private double wizardRadius, wizardRadiusSq;

        public CellStatus[,] cells;

        // Nodes
        // ... empty

        public Physic(Wizard self, World world, Game game, Move move)
        {
            this.self = self;
            this.world = world;
            this.game = game;
            this.move = move;

            wizardRadius = self.Radius;
            wizardRadiusSq = wizardRadius*wizardRadius;

            InitMap();
        }

        private void InitMap()
        {
            GenerateClusters();
            GenerateCells();
        }

        private void GenerateClusters()
        {
            ClusterGridCount = (int)Math.Ceiling(game.MapSize / ClusterGridWidth);

            clusters = new List<CircularUnit>[ClusterGridCount, ClusterGridCount];
            for (int i = 0; i < ClusterGridCount; i++)
            {
                for (int j = 0; j < ClusterGridCount; j++)
                {
                    clusters[i, j] = new List<CircularUnit>();
                }
            }

            // Fill
            AddUnitsToClusters(world.Buildings);
            AddUnitsToClusters(world.Minions);
            AddUnitsToClusters(world.Trees);
            AddUnitsToClusters(world.Wizards.Where(wizard => !wizard.IsMe));

            basesBuildings = world.Buildings.Where(b => b.Type == BuildingType.FactionBase).ToArray();
        }

        private void GenerateCells()
        {
            CellGridCount = (int)Math.Ceiling(game.MapSize / CellGridWidth);

            cells = new CellStatus[CellGridCount,CellGridCount];
        }

        private void AddUnitsToClusters(IEnumerable<CircularUnit> units)
        {
            foreach (var unit in units)
            {
                var p = GetCluster(unit);
                clusters[p.X, p.Y].Add(unit);
            }
        }

        public Path<QNode> FindPathTo(VectorD end)
        {
            return FindPath(GetCell(self), GetCell(end));
        }

        public Path<QNode> FindPath(VectorD start, VectorD end)
        {
            return FindPath(GetCell(start), GetCell(end));
        }

        public Path<QNode> FindPath(Vector start, Vector end)
        {
            var startNode = GetNode(start);
            var endNode = GetNode(end);

            Path<QNode> shortestPath = AStar.FindPath(startNode, endNode, 
                (node1, node2) => (node2.Vec - node1.Vec).Length, 
                node => (endNode.Vec - node.Vec).Length,
                20);

            return shortestPath;
        }

        public QNode GetNode(Vector cell)
        {
            return new QNode(cell, this, null);
        }

        public bool IsFreeCell(VectorD coord)
        {
            return IsFreeCell(GetCell(coord));
        }

        public bool IsFreeCell(Vector cell)
        {
            if (cells[cell.X, cell.Y] == CellStatus.Unknown)
            {
                cells[cell.X, cell.Y] = CalculateCellStatus(cell);
            }

            return cells[cell.X, cell.Y] == CellStatus.Free;
        }

        public CellStatus CalculateCellStatus(Vector cell)
        {
            var coords = GetPointFromCell(cell);

            var centralCluster = GetCluster(coords);

            foreach (var neigh in neisAndMeClusters)
            {
                var cl = (centralCluster + neigh);

                if (cl.X < 0 || cl.Y < 0 || cl.X >= ClusterGridCount || cl.Y >= ClusterGridCount)
                    continue;

                foreach (var unit in clusters[cl.X, cl.Y])
                {
                    var unitPointRadius = (wizardRadius + unit.Radius);
                    if (SquareDistance(unit, coords) <= unitPointRadius * unitPointRadius)
                    {
                        return CellStatus.Busy;
                    }
                }

                foreach (var building in basesBuildings)
                {
                    var unitPointRadius = (wizardRadius + building.Radius);
                    if (SquareDistance(building, coords) <= unitPointRadius * unitPointRadius)
                    {
                        return CellStatus.Busy;
                    }
                }
            }

            return CellStatus.Free;
        }

        public Vector GetCluster(Unit unit)
        {
            return new Vector((int)Math.Floor(unit.X / ClusterGridWidth), (int)Math.Floor(unit.Y / ClusterGridWidth));
        }

        public Vector GetCluster(VectorD coord)
        {
            return new Vector((int)Math.Floor(coord.X / ClusterGridWidth), (int)Math.Floor(coord.Y / ClusterGridWidth));
        }

        public Vector GetCell(Unit unit)
        {
            return new Vector((int)Math.Floor(unit.X / CellGridWidth), (int)Math.Floor(unit.Y / CellGridWidth));
        }

        public Vector GetCell(VectorD coord)
        {
            return new Vector((int)Math.Floor(coord.X / CellGridWidth), (int)Math.Floor(coord.Y / CellGridWidth));
        }

        public VectorD GetPointFromCell(Vector cell)
        {
            return new VectorD(cell.X * CellGridWidth + CellGridWidth / 2, cell.Y * CellGridWidth + CellGridWidth / 2);
        }

        public double SquareDistance(Unit unit1, Unit unit2)
        {
            return new VectorD(unit1.X - unit2.X, unit1.Y - unit2.Y).SqLength;
        }

        public double SquareDistance(Unit unit, VectorD coord)
        {
            return new VectorD(coord.X - unit.X, coord.Y - unit.Y).SqLength;
        }

        public enum CellStatus
        {
            Unknown,
            Free,
            Busy
        }
    }

    public class QCluster<TNode> where TNode : IHasNeighbours<TNode>
    {
        public List<TNode> Nodes { get; set; } = new List<TNode>();
    }

    public class QNode : IHasNeighbours<QNode>, IEquatable<QNode>
    {
        //public Func<QNode, IEnumerable<QNode>> CalculateNeighbours { get; }
        public Physic Ph { get; }
        public Vector Vec { get; }

        private IEnumerable<QNode> _neighbours;
        public IEnumerable<QNode> Neighbours
        {
            get
            {
                if (_neighbours == null)
                {
                    _neighbours = CalculateNeighbours();
                }

                return _neighbours;
            }
            private set { _neighbours = value; }
        }

        public QNode(Vector vec, Physic physic, IEnumerable<QNode> neighbours)
        {
            Vec = vec;
            //CalculateNeighbours = calculateNeighbours;
            Ph = physic;
            Neighbours = neighbours;
        }

        public IEnumerable<QNode> CalculateNeighbours()
        {
            var centralCell = Vec;

            List<QNode> neighbours = new List<QNode>();

            foreach (var neighbour in Physic.neighboursVectors)
            {
                var considerCell = (centralCell + neighbour);

                if (considerCell.X < 0 || considerCell.Y < 0 || considerCell.X >= Ph.CellGridCount || considerCell.Y >= Ph.CellGridCount)
                    continue;

                if (Ph.IsFreeCell(considerCell))
                {
                    neighbours.Add(new QNode(considerCell, Ph, null));
                }
            }

            return neighbours;
        }

        #region IEquatable<QNode>
        public bool Equals(QNode other)
        {
            if (ReferenceEquals(null, other)) return false;
            if (ReferenceEquals(this, other)) return true;
            return Equals(Vec, other.Vec);
        }

        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj)) return false;
            if (ReferenceEquals(this, obj)) return true;
            if (obj.GetType() != this.GetType()) return false;
            return Equals((QNode) obj);
        }

        public override int GetHashCode()
        {
            return (Vec != null ? Vec.GetHashCode() : 0);
        }

        public static bool operator ==(QNode left, QNode right)
        {
            return Equals(left, right);
        }

        public static bool operator !=(QNode left, QNode right)
        {
            return !Equals(left, right);
        }
        #endregion IEquatable<QNode>
    }

    #region AStar Implementation
    public class AStar
    {
        public static Dictionary<Vector, Vector> SavedVectors = new Dictionary<Vector, Vector>();
        public static Dictionary<Vector, Vector> ToSaveVectors = new Dictionary<Vector, Vector>();

        public static Path<TNode> FindPath<TNode>(
            TNode start,
            TNode destination,
            Func<TNode, TNode, double> distance,
            Func<TNode, double> estimate,
            int breakAtPathLength = -1) where TNode : QNode
        {
            var closed = new HashSet<TNode>();

            var queue = new PriorityQueue<double, Path<TNode>>();

            queue.Enqueue(0, new Path<TNode>(start));

            while (!queue.IsEmpty)
            {
                var path = queue.Dequeue();

                if (closed.Contains(path.LastStep))
                    continue;

                if (path.LastStep.Equals(destination))
                    return path;

                if (breakAtPathLength > 0 && path.PathLength == breakAtPathLength)
                {
                    //ToSaveVectors.Add(destination.Vec, path.LastStep.Vec);
                    return path;
                    //List<Path<TNode>> pathList = new List<Path<TNode>>();
                    //pathList.Add(path);

                    //while (!queue.IsEmpty)
                    //{
                    //    Path<TNode> p = queue.Dequeue();

                    //    if (p.TotalCost != path.TotalCost)
                    //        break;

                    //    pathList.Add(p);
                    //}

                    //return pathList
                    //    .OrderBy(p => p.LastStep.Vec.X)
                    //    .ThenBy(p => p.LastStep.Vec.Y)
                    //    .Last();
                }

                closed.Add(path.LastStep);

                foreach (TNode n in path.LastStep.Neighbours)
                {
                    double d = distance(path.LastStep, n);

                    Vector savedVector;
                    if (SavedVectors.TryGetValue(destination.Vec, out savedVector) && n.Vec == savedVector)
                    {
                        d = -10;
                    }

                    var newPath = path.AddStep(n, d);

                    queue.Enqueue(newPath.TotalCost + estimate(n), newPath);
                }
            }

            return null;
        }
    }

    public interface IHasNeighbours<N>
    {
        IEnumerable<N> Neighbours { get; }
    }

    public class Path<TNode> : IEnumerable<Path<TNode>>
    {
        public int PathLength { get; private set; }
        public TNode LastStep { get; private set; }
        public Path<TNode> PreviousSteps { get; private set; }
        public double TotalCost { get; private set; }

        private Path(TNode lastStep, Path<TNode> previousSteps, double totalCost, int pathLength)
        {
            PathLength = pathLength;
            LastStep = lastStep;
            PreviousSteps = previousSteps;
            TotalCost = totalCost;
        }

        public Path(TNode start) : this(start, null, 0, 1) { }

        public Path<TNode> AddStep(TNode step, double stepCost)
        {
            return new Path<TNode>(step, this, TotalCost + stepCost, PathLength + 1);
        }

        public IEnumerator<Path<TNode>> GetEnumerator()
        {
            for (Path<TNode> p = this; p != null; p = p.PreviousSteps)
                yield return p;
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
    }

    public class PriorityQueue<P, V> : IEnumerable
    {
        private SortedDictionary<P, Queue<V>> list = new SortedDictionary<P, Queue<V>>();

        public void Enqueue(P priority, V value)
        {
            Queue<V> q;

            if (!list.TryGetValue(priority, out q))
            {
                q = new Queue<V>();

                list.Add(priority, q);
            }

            q.Enqueue(value);
        }

        public V Dequeue()
        {
            // will throw if there isn’t any first element!
            var pair = list.First();

            var v = pair.Value.Dequeue();

            if (pair.Value.Count == 0) // nothing left of the top priority.
                list.Remove(pair.Key);

            return v;
        }

        public bool IsEmpty
        {
            get { return !list.Any(); }
        }

        #region IEnumerable Members

        IEnumerator IEnumerable.GetEnumerator()
        {
            return list.GetEnumerator();
        }

        #endregion
    }
    #endregion

    public class Task
    {
        public Move Movement { get; } = new Move();
        public double Cost { get; } = 0;
    }

    #region Math
    public class QMath
    {
        public static VectorD AngleToVector(double angle)
        {
            // It's works only for Russia AI Cup physical
            return new VectorD(Math.Cos(angle), Math.Sin(angle));
        }

        public static double VectorToAngle(VectorD vec)
        {
            return Math.Atan2(vec.Y, vec.X);
        }

        public static double Clamp(double value, double min, double max)
        {
            if (value > max)
                return max;
            if (value < min)
                return min;

            return value;
        }

        public static double AngleNormalize(double angle)
        {
            while (angle > Math.PI)
            {
                angle -= 2.0D * Math.PI;
            }

            while (angle < -Math.PI)
            {
                angle += 2.0D * Math.PI;
            }

            return angle;
        }
    }

    public class Vector : IEquatable<Vector>
    {
        public int X { get; set; }
        public int Y { get; set; }

        public Vector(int x, int y)
        {
            X = x;
            Y = y;
        }

        public Vector()
        {
        }

        public static Vector operator +(Vector a, Vector b)
        {
            return new Vector(a.X + b.X, a.Y + b.Y);
        }

        public static Vector operator -(Vector a, Vector b)
        {
            return new Vector(a.X - b.X, a.Y - b.Y);
        }

        public static int operator *(Vector a, Vector b)
        {
            return a.X * b.X + a.Y * b.Y;
        }

        public static bool operator ==(Vector a, Vector b)
        {
            if (((object)a == null) || ((object)b == null))
                return false;

            return a.Equals(b);
        }

        public static bool operator !=(Vector a, Vector b)
        {
            return !(a == b);
        }

        public double Length
        {
            get
            {
                return Math.Sqrt(X * X + Y * Y);
            }
        }
        public int SqrLength
        {
            get
            {
                return X * X + Y * Y;
            }
        }

        public bool Equals(Vector other)
        {
            if ((object)other == null) return false;

            return this.X == other.X && this.Y == other.Y;
        }

        public override bool Equals(object obj)
        {
            if (obj is Vector)
            {
                return this.Equals(obj as Vector);
            }

            return false;
        }

        public override int GetHashCode()
        {
            return X.GetHashCode() ^ (Y.GetHashCode() * 1005001);
        }

        public override string ToString()
        {
            return string.Format("({0}, {1})", X, Y);
        }

        public Vector Clone()
        {
            return new Vector(X, Y);
        }
    }

    public class VectorD
    {
        public double X { get; set; }
        public double Y { get; set; }

        public VectorD(double x, double y)
        {
            X = x;
            Y = y;
        }

        public VectorD(Vector v)
        {
            X = v.X;
            Y = v.Y;
        }

        public VectorD(VectorD v)
        {
            X = v.X;
            Y = v.Y;
        }

        public VectorD(Unit u)
            : this(u.X, u.Y)
        {
        }

        public double GetAngleTo(VectorD vec)
        {
            double vecAngel = Math.Atan2(vec.Y, vec.X);
            double thisAngel = Math.Atan2(this.Y, this.X);
            double relativeAngleTo = vecAngel - thisAngel;

            while (relativeAngleTo > Math.PI)
            {
                relativeAngleTo -= 2.0D * Math.PI;
            }

            while (relativeAngleTo < -Math.PI)
            {
                relativeAngleTo += 2.0D * Math.PI;
            }

            return relativeAngleTo;
        }

        public static VectorD operator +(VectorD a, VectorD b)
        {
            return new VectorD(a.X + b.X, a.Y + b.Y);
        }

        public static VectorD operator -(VectorD a, VectorD b)
        {
            return new VectorD(a.X - b.X, a.Y - b.Y);
        }

        public static double operator *(VectorD a, VectorD b)
        {
            return a.X * b.X + a.Y * b.Y;
        }

        public static VectorD operator *(VectorD a, double b)
        {
            return new VectorD(a.X * b, a.Y * b);
        }

        public double VectorMul(VectorD b)
        {
            return (this.X * b.Y - b.X * this.Y); // Return Z axis
        }

        public double Length => Math.Sqrt(X * X + Y * Y);

        public double SqLength => X * X + Y * Y;

        public VectorD Normalize
        {
            get
            {
                double length = Length;

                if (length == 0)
                {
                    return new VectorD(X, Y);
                }

                length = 1.0D / length;

                return new VectorD(X * length, Y * length);
            }
        }

        public override string ToString()
        {
            return string.Format("({0}, {1})", X, Y);
        }
    }
    #endregion

#if QDEBUG
    public sealed class VisualClient
    {
        private const int BUFFER_SIZE_BYTES = 1 << 24;

        private readonly TcpClient _client;
        private readonly BinaryWriter _writer;

        public VisualClient(string host, int port)
        {
            _client = new TcpClient(host, port)
            {
                SendBufferSize = BUFFER_SIZE_BYTES,
                ReceiveBufferSize = BUFFER_SIZE_BYTES,
                NoDelay = true
            };

            _writer = new BinaryWriter(_client.GetStream());

            CultureInfo newCInfo = (CultureInfo)Thread.CurrentThread.CurrentCulture.Clone();
            newCInfo.NumberFormat.NumberDecimalSeparator = ".";
            Thread.CurrentThread.CurrentCulture = newCInfo;
        }

        //---------------------------------------------------------------------
        // Public
        //---------------------------------------------------------------------

        /// <summary>
        /// Start queueing commands to be displayed either before main drawing
        /// </summary>
        public void BeginPre()
        {
            SendCommand("begin pre");
        }

        /// <summary>
        /// Start queueing commands to be displayed either after main drawing
        /// </summary>
        public void BeginPost()
        {
            SendCommand("begin post");
        }

        /// <summary>
        /// start queueing commands to be displayed on the absolute coordinates
        /// </summary>
        public void beginAbs()
        {
            SendCommand("begin abs");
        }

        /// <summary>
        /// Mark either "pre" queue of commands as ready to be displayed
        /// </summary>
        public void EndPre()
        {
            SendCommand("end pre");
        }

        /// <summary>
        /// Mark either "post" queue of commands as ready to be displayed
        /// </summary>
        public void EndPost()
        {
            SendCommand("end post");
        }

        /// <summary>
        // mark either "abs" queue of commands as ready to be displayed
        /// <summary>
        public void endAbs()
        {
            SendCommand("end abs");
        }


        /// <summary>
        /// Draw a circle at (x, y) with radius r and color color
        /// </summary>
        public void Circle(double x, double y, float radius, float r = 0f, float g = 0f, float b = 0f)
        {
            ValidateColor(r, g, b);

            SendCommand($"circle {x} {y} {radius} {r} {g} {b}");
        }

        /// <summary>
        /// Draw a filled circle at (x, y) with radius r and color color
        /// </summary>
        public void FillCircle(double x, double y, float radius, float r = 0f, float g = 0f, float b = 0f)
        {
            ValidateColor(r, g, b);

            SendCommand($"fill_circle {x} {y} {radius} {r} {g} {b}");
        }

        /// <summary>
        /// Draw a rect with corners at (x, y) to (x, y) with color color
        /// </summary>
        public void Rect(double x1, double y1, double x2, double y2, float r = 0f, float g = 0f, float b = 0f)
        {
            ValidateColor(r, g, b);

            SendCommand($"rect {x1} {y1} {x2} {y2} {r} {g} {b}");
        }

        /// <summary>
        /// Draw a filled rect with corners at (x1, y1) to (x2, y2) with color color
        /// </summary>
        public void FillRect(double x1, double y1, double x2, double y2, float r = 0f, float g = 0f, float b = 0f)
        {
            ValidateColor(r, g, b);

            SendCommand($"fill_rect {x1} {y1} {x2} {y2} {r} {g} {b}");
        }

        /// <summary>
        /// Draw a line from (x1, y1) to (x2, y2) with color color
        /// </summary>
        public void Line(double x1, double y1, double x2, double y2, float r = 0f, float g = 0f, float b = 0f)
        {
            ValidateColor(r, g, b);

            SendCommand($"line {x1} {y1} {x2} {y2} {r} {g} {b}");
        }

        /// <summary>
        /// Show msg at coordinates (x, y) with color color
        /// </summary>
        public void Text(double x, double y, string msg, float r = 0f, float g = 0f, float b = 0f)
        {
            ValidateColor(r, g, b);

            SendCommand($"text {x} {y} {msg} {r} {g} {b}");
        }

        /// <summary>
        /// draw a arc with center at (x, y), radius radius and angle arcAngle, started from startAngle with color color, angles in radians
        /// < /summary>
        public void Arc(double x, double y, double radius, double startAngle, double arcAngle, float r = 0f, float g = 0f, float b = 0f)
        {
            ValidateColor(r, g, b);
            SendCommand($"arc {x} {y} {radius} {startAngle} {arcAngle} {r} {g} {b}");
        }

        /// <summary>
        /// draw a filled arc with center at (x, y), radius radius and angle arcAngle, started from startAngle with color color, angles in radians
        /// </summary>
        public void FillArc(double x, double y, double radius, double startAngle, double arcAngle, float r = 0f, float g = 0f, float b = 0f)
        {
            ValidateColor(r, g, b);
            SendCommand($"fill_arc {x} {y} {radius} {startAngle} {arcAngle} {r} {g} {b}");
        }


        //---------------------------------------------------------------------
        // Helpers
        //---------------------------------------------------------------------

        private void SendCommand(string command)
        {
            WriteString(command + "\n");
            _writer.Flush();
        }

        private void WriteString(string value)
        {
            if (value == null)
            {
                WriteInt(-1);
                return;
            }

            var bytes = Encoding.UTF8.GetBytes(value);
            _writer.Write(bytes, 0, bytes.Length);
        }

        private void WriteInt(int value)
        {
            _writer.Write(value);
        }

        public void Close()
        {
            _client.Close();
        }

        private static void ValidateColor(float r, float g, float b)
        {
            const string outOfRangeMessage = "The color value must be in range from 0.0 to 1.0 .";
            if (r < 0 || r > 1)
            {
                throw new ArgumentOutOfRangeException(nameof(r), outOfRangeMessage);
            }

            if (g < 0 || g > 1)
            {
                throw new ArgumentOutOfRangeException(nameof(g), outOfRangeMessage);
            }

            if (b < 0 || b > 1)
            {
                throw new ArgumentOutOfRangeException(nameof(b), outOfRangeMessage);
            }
        }
    }
#endif

    public static class MyExtensions
    {
        public static void CopyFrom(this Move my, Move from)
        {
            my.Turn = from.Turn;
            my.Speed = from.Speed;
            my.StrafeSpeed = from.StrafeSpeed;
            my.Action = from.Action;
            my.CastAngle = from.CastAngle;
            my.MaxCastDistance = from.MaxCastDistance;
            my.MinCastDistance = from.MinCastDistance;
            my.SkillToLearn = from.SkillToLearn;
            my.Messages = from.Messages;
            my.StatusTargetId = from.StatusTargetId;
        }
    }
}