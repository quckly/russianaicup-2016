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


        // For one move variables
        Wizard self; World world; Game game; Move move;
        private Physic physic;
        private List<Task> moveTasks;

        public void CalculateNextMovement()
        {

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

            move = bestTask.Movement;
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
                    var free = physic.IsFreeCell(new Vector(i, j));

                    if (free)
                        vc.FillCircle(coords.X, coords.Y, 5.0f, 0, 1, 0);
                    else
                        vc.FillCircle(coords.X, coords.Y, 5.0f, 1, 0, 0);
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

        private static readonly Vector[] neighboursVectors = {new Vector(-1, -1), new Vector(0, -1), new Vector(1, -1), new Vector(-1, 0), new Vector(1, 0), new Vector(-1, 1), new Vector(0, 1), new Vector(1,1) };
        private static readonly Vector[] neighboursClusters = neighboursVectors.Union(new[] {new Vector(0, 0)}).ToArray();

        // Cells
        // {1,2,4,5,8,10,16,20,25,32,40,50,80,100,125,160,200,250,400,500,800,1000,2000,4000}
        public readonly int CellGridWidth = 20;
        public int CellGridCount { get; private set; }

        private double wizardRadius, wizardRadiusSq;

        private CellStatus[,] cells;

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

        public Path<QNode> FindPath(VectorD start, VectorD end)
        {
            return null;
        }

        public Path<QNode> FindPath(Vector start, Vector end)
        {
            return null;//Path<QNode> shortestPath = AStar.FindPath(start, destination, distance, haversineEstimation);
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
            var coords = new VectorD(cell.X * CellGridWidth + CellGridWidth / 2, cell.Y * CellGridWidth + CellGridWidth / 2);

            var centralCluster = GetCluster(coords);

            foreach (var neigh in neighboursClusters)
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
            return new Vector((int)Math.Floor(unit.X / ClusterGridWidth), (int)Math.Floor(unit.Y / ClusterGridWidth));
        }

        public Vector GetCell(VectorD coord)
        {
            return new Vector((int)Math.Floor(coord.X / ClusterGridWidth), (int)Math.Floor(coord.Y / ClusterGridWidth));
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

    public class QNode : IHasNeighbours<QNode>
    {
        public QNode PathParent { get; }
        public int X { get; }
        public int Y { get; }
        public IEnumerable<QNode> Neighbours { get; }

        public QNode(QNode pathParent, int x, int y, IEnumerable<QNode> neighbours)
        {
            PathParent = pathParent;
            X = x;
            Y = y;
            Neighbours = neighbours;
        }
    }

    #region AStar Implementation
    public class AStar
    {
        static public Path<TNode> FindPath<TNode>(
            TNode start,
            TNode destination,
            Func<TNode, TNode, double> distance,
            Func<TNode, double> estimate) where TNode : IHasNeighbours<TNode>
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

                closed.Add(path.LastStep);

                foreach (TNode n in path.LastStep.Neighbours)
                {
                    double d = distance(path.LastStep, n);

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
        public TNode LastStep { get; private set; }

        public Path<TNode> PreviousSteps { get; private set; }

        public double TotalCost { get; private set; }

        private Path(TNode lastStep, Path<TNode> previousSteps, double totalCost)
        {
            LastStep = lastStep;

            PreviousSteps = previousSteps;

            TotalCost = totalCost;
        }

        public Path(TNode start) : this(start, null, 0) { }

        public Path<TNode> AddStep(TNode step, double stepCost)
        {
            return new Path<TNode>(step, this, TotalCost + stepCost);
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
        public Move Movement { get; }
        public double Cost { get; }
    }

    #region Math
    public class QMath
    {
        public static VectorD AngleToVector(double angle)
        {
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
}