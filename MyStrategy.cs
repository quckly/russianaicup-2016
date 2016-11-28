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
        private VectorD NextPoint = new VectorD(4000-250, 250);

        BonusManager bonusManager = new BonusManager();
        private int nextSkillToLearn = 0;

        // For one move variables
        Wizard self; World world; Game game; Move move;
        private CachedResources cached;
        public static Physic physic;
        private List<Task> moveTasks;

        // Consts:
        public class Const
        {
            public static readonly double LOW_HP_FACTOR = 0.35;
            public static readonly double LOW_HP_TASK_MUL = 150;

            public static readonly double COST_ENEMY_ATTACK = 4000;
            public static readonly double ENEMY_ATTACK_VOLATILE = 2000;
            public static readonly double ATTACK_ENEMY_RADIUS_PART = 0.5;
            public static readonly double ATTACK_ENEMY_RADIUS_PART_BUILDING = 0.95;

            public static readonly double COST_ENEMY_ATTACK_PRE = 1500;
            public static readonly double ENEMY_ATTACK_VOLATILE_PRE = 500;

            public static readonly double CONSIDER_ATTACK_DISTANCE = 150;
            public static readonly double fddfb = 110;
            public static readonly double fdbfdb = 110;
            public static readonly double fdb = 110;
            public static readonly double fdbf = 110;

            public static readonly VectorD BaseOrigin = new VectorD(250, 4000-250);
            public static readonly VectorD[] BonusesOrigin = { new VectorD(1200, 1200), new VectorD(2800, 2800) };

            public static readonly SkillType[] SkillsToLearn =
            {
                SkillType.RangeBonusPassive1, SkillType.RangeBonusAura1,
                SkillType.RangeBonusPassive2, SkillType.RangeBonusAura2,
                SkillType.AdvancedMagicMissile,

                SkillType.MagicalDamageAbsorptionPassive1, SkillType.MagicalDamageAbsorptionAura1,
                SkillType.MagicalDamageAbsorptionPassive2, SkillType.MagicalDamageAbsorptionAura2,

                SkillType.MagicalDamageBonusPassive1, SkillType.MagicalDamageBonusAura1,
                SkillType.MagicalDamageBonusPassive2, SkillType.MagicalDamageBonusAura2
            };
        }

        /***
         * 
         * Move to point = 1 000
         * 
         * Backtrack: 1500-2000
         * Towers: 2000-3000
         * Attack 4000-6000, Wizard: 8000-12000
         * 
         * Low HP go back ~ 10 000
         * 
         */
        public void CalculateNextMovement()
        {
            CalculateLowHpStrategy();
            CalculateAttack();

            CalulatePickUpBonuses();

            RunToMidlle();
        }

        public void CalculateLowHpStrategy()
        {
            if (self.Life < self.MaxLife * Const.LOW_HP_FACTOR)
            {
                var task = new Task(TaskType.MoveTo);
                task.Cost = (self.MaxLife - self.Life) * Const.LOW_HP_TASK_MUL;

                task.Target = Const.BaseOrigin;

                AddTask(task);
            }
        }

        private void CalculateAttack()
        {
            // ���� � ������� ���������
            if (self.RemainingActionCooldownTicks == 0)
            {
                if (cached.RemainingCooldownTicksByAction[(int)ActionType.MagicMissile] == 0 && self.Mana >= game.MagicMissileManacost)
                {
                    foreach (var enemy in cached.Enemies)
                    {
                        var radiusPart = enemy is Building ? Const.ATTACK_ENEMY_RADIUS_PART_BUILDING : Const.ATTACK_ENEMY_RADIUS_PART;

                        var distanceToEnemy = self.GetDistanceTo(enemy) - enemy.Radius * radiusPart;
                        if (distanceToEnemy > self.CastRange)
                            continue;

                        double angle = self.GetAngleTo(enemy);

                        var task = new Task();
                        task.Movement.Turn = angle;

                        // ���� ���� ����� ����, ...
                        if (Math.Abs(angle) < game.StaffSector/(2 + 0.1))
                        {
                            task.Movement.Action = ActionType.MagicMissile;
                            task.Movement.CastAngle = angle;
                            task.Movement.MinCastDistance = (distanceToEnemy - enemy.Radius * (1 - radiusPart) + game.MagicMissileRadius);

                            task.Cost = Const.COST_ENEMY_ATTACK +
                                        (1 - (double) enemy.Life/enemy.MaxLife)*Const.ENEMY_ATTACK_VOLATILE;
                        }

                        if (enemy is Wizard)
                        {
                            task.Cost *= 2;
                        }
                        else if (enemy is Minion)
                        {

                        }
                        else if (enemy is Building)
                        {
                            task.Cost *= 0.5;
                        }

                        AddTask(task);
                    }
                }
            }

            // ����������� �����
            var considerAttackDistance = self.CastRange + Const.CONSIDER_ATTACK_DISTANCE;

            foreach (var enemy in cached.Enemies)
            {
                var distanceToEnemy = self.GetDistanceTo(enemy);
                if (distanceToEnemy > considerAttackDistance)
                    continue;

                double angle = self.GetAngleTo(enemy);

                var task = new Task(TaskType.BackTrack)
                {
                    Movement = {Turn = angle},
                    Cost = Const.COST_ENEMY_ATTACK_PRE + (1 - (double) enemy.Life/enemy.MaxLife)*Const.ENEMY_ATTACK_VOLATILE_PRE
                };

                if (enemy is Wizard)
                {
                    task.Cost *= 2;
                }

                // Calculate next point
                var direction = (Const.BaseOrigin - new VectorD(enemy)).Normalize;
                var radiusPart = enemy is Building ? Const.ATTACK_ENEMY_RADIUS_PART_BUILDING : Const.ATTACK_ENEMY_RADIUS_PART;
                var attackRange = Math.Max(self.CastRange, GetAttackRange(enemy));
                var distance = enemy.Radius * radiusPart + attackRange;
                var tickDelay = Math.Max(self.RemainingActionCooldownTicks, cached.RemainingCooldownTicksByAction[(int)ActionType.MagicMissile]);

                if (tickDelay - 1 > 0)
                {
                    distance += Math.Min(10.0, (tickDelay-1) * 3);
                }
                else
                {
                    distance -= 10.0;
                    distance -= Math.Max(0, GetAttackRange(enemy) - self.CastRange);
                    //task.Cost = 1100;
                }
                
                task.Target = new VectorD(enemy) + direction * distance;

                AddTask(task);
            }
        }

        private void CalulatePickUpBonuses()
        {
            while (bonusManager.MayBeBonus)
            {
                var selfVec = new VectorD(self);

                var nearestBonus = bonusManager.Bonuses
                    .Where(b => b.Avaliable)
                    .Min(b => (b.Origin - selfVec).SqLength);

                if ((nearestBonus.Origin - selfVec).Length <= game.WizardVisionRange)
                {
                    var bonusExists = cached.Bonuses.Any(bonus => (new VectorD(bonus) - nearestBonus.Origin).Length <= 5);

                    if (!bonusExists)
                    {
                        nearestBonus.Avaliable = false;
                        break;
                    }
                }

                AddTask(new Task(TaskType.MoveTo) {Cost = 3500, Target = nearestBonus.Origin});
                return;
            }
        }

        private void RunToMidlle()
        {
            AddTask(new Task(TaskType.MoveTo) {Target = NextPoint});
        }

        private VectorD CalculatePathToTarget(VectorD targetPoint)
        {
            var path = physic.FindPathTo(targetPoint);
            var result = NextPoint;

            if (path != null)
            {
                var pa = path.Reverse().ToArray();

                if (pa.Length > 1)
                    result = physic.GetPointFromCellLocal(pa[1].LastStep.Vec);

#if QDEBUG
                foreach (var ppp in pa)
                {
                    var pp = physic.GetPointFromCellLocal(ppp.LastStep.Vec);
                    vc.FillCircle(pp.X, pp.Y, 4.0f, 0, 0, 1);
                }
#endif
            }

            return result;
        }

        private double GetAttackRange(LivingUnit unit)
        {
            if (unit is Wizard)
            {
                return (unit as Wizard).CastRange;
            }
            if (unit is Building)
            {
                return (unit as Building).AttackRange;
            }
            if (unit is Minion)
            {
                return (unit as Minion).Type == MinionType.OrcWoodcutter
                    ? game.OrcWoodcutterAttackRange
                    : game.FetishBlowdartAttackRange;
            }

            return 0;
        }

        private void SetRunToPoint(Move move, VectorD target, bool changeTurn = true)
        {
            var vectorToTarget = (target - new VectorD(self));
            var vectorToTargetLength = vectorToTarget.Length;

            if (vectorToTargetLength < 2)
            {
                move.Speed = move.StrafeSpeed = 0;
                return;
            }

            double angle = self.GetAngleTo(target.X, target.Y);

            VectorD vect = QMath.AngleToVector(angle);

            double coof = new VectorD(vect.X / 4.0, vect.Y / 3.0).Length;
            vect *= 1.0 / coof;

            // Clamp speed vector
            var vectLen = vect.Length;
            if (vectLen > vectorToTargetLength)
            {
                vect = vect.Normalize * vectorToTargetLength;
            }

            move.StrafeSpeed = vect.Y;
            move.Speed = vect.X;

            //vc.Line(self.X, self.Y, (self.X + vect.X), (self.Y + vect.Y), 0.7f, 0.9f, 0.1f);

            if (changeTurn)
            {
                move.Turn = angle;
            }
        }

        private void AddTask(Task task)
        {
            moveTasks.Add(task);
        }

        private void ApplyNextMovement()
        {
            var bestTask = moveTasks
                .OrderBy(task => -task.Cost)
                .FirstOrDefault();

            if (bestTask == null) // This is not be true never
            {
                move.Action = ActionType.Staff;
                return;
            }

            if (bestTask.Type == TaskType.MoveTo)
            {
                var target = CalculatePathToTarget(bestTask.Target);

                SetRunToPoint(move, target);
            }
            else if (bestTask.Type == TaskType.BackTrack)
            {
                var target = CalculatePathToTarget(bestTask.Target);

                move.CopyFrom(bestTask.Movement);
                SetRunToPoint(move, target, false);
            }
            else
            {
                move.CopyFrom(bestTask.Movement);
            }

            LearnSkills();
        }

        private void LearnSkills()
        {
            if (nextSkillToLearn >= Const.SkillsToLearn.Length)
                return;

            if (self.Skills.Contains(Const.SkillsToLearn[nextSkillToLearn]))
                nextSkillToLearn++;

            move.SkillToLearn = Const.SkillsToLearn[nextSkillToLearn];
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

            cached = new CachedResources(self, world);

            physic = new Physic(self, world, game, move, cached);

            moveTasks = new List<Task>();

            //
            bonusManager.WorldTick(world);

            // Dirty hacks
            AStar.SavedVectors = AStar.ToSaveVectors;
            AStar.ToSaveVectors = new Dictionary<Vector, Vector>();
        }

        #region DEBUG
#if QDEBUG
        private static bool tryConnect = false;
        public static VisualClient vc = null;

        private void DrawCells()
        {
            for (int i = 0; i < physic.LocalCellGridCount; i++)
            {
                for (int j = 0; j < physic.LocalCellGridCount; j++)
                {
                    var coords = new VectorD(i * physic.LocalCellGridWidth + physic.LocalCellGridWidth / 2, j * physic.LocalCellGridWidth + physic.LocalCellGridWidth / 2);
                    var free = physic.cellsLocal[i, j];//physic.IsFreeCell(new Vector(i, j));

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
            //DrawCells();

            vc.EndPost();
        }
#endif
        #endregion
    }

    public class Task
    {
        public TaskType Type { get; set; } = TaskType.Raw;
        public double Cost { get; set; } = 0;

        public Move Movement { get; set; } = new Move();
        public VectorD Target { get; set; }

        public Task() { }

        public Task(TaskType type, double cost, VectorD target)
        {
            Type = type;
            Cost = cost;
            Target = target;
        }

        public Task(TaskType type)
        {
            Type = type;
        }
    }

    public enum TaskType
    {
        Raw = 0,
        MoveTo,
        BackTrack
    }

    public class CachedResources
    {
        Wizard wizard; World world;

        // Self
        public SkillType[] Skills { get; }
        public int[] RemainingCooldownTicksByAction { get; }

        // World
        public Player[] Players { get; }
        public Wizard[] Wizards { get; }
        public Minion[] Minions { get; }
        public Projectile[] Projectiles { get; }
        public Bonus[] Bonuses { get; }
        public Building[] Buildings { get; }
        public Tree[] Trees { get; }

        // Common
        public LivingUnit[] Enemies { get; }

        public CachedResources(Wizard wizard, World world)
        {
            this.wizard = wizard;
            this.world = world;

            Skills = wizard.Skills;
            RemainingCooldownTicksByAction = wizard.RemainingCooldownTicksByAction;

            Players = world.Players;
            Wizards = world.Wizards;
            Minions = world.Minions;
            Projectiles = world.Projectiles;
            Bonuses = world.Bonuses;
            Buildings = world.Buildings;
            Trees = world.Trees;

            Enemies = GetEnemies().ToArray();
        }
        
        private List<LivingUnit> GetEnemies()
        {
            var enemies = new List<LivingUnit>();

            AddEnemies(enemies, Minions);
            AddEnemies(enemies, Buildings);
            AddEnemies(enemies, Wizards);

            return enemies;
        }

        private void AddEnemies(List<LivingUnit> enemies, LivingUnit[] candidates)
        {
            foreach (var unit in candidates)
            {
                if (unit.Faction == wizard.Faction || unit.Faction == Faction.Neutral)
                    continue;

                enemies.Add(unit);
            }
        }
    }

    public class BonusManager
    {
        public QBonus[] Bonuses { get; } = {
            new QBonus(MyStrategy.Const.BonusesOrigin[0]),
            new QBonus(MyStrategy.Const.BonusesOrigin[1])
        };
        
        public bool MayBeBonus => Bonuses.Any(bonus => bonus.Avaliable);
        public QBonus TargetBonus { get; set; }

        public void WorldTick(World world)
        {
            if (world.TickIndex > 0 && world.TickIndex % 2500 == 0)
            {
                foreach (var bonus in Bonuses)
                    bonus.Avaliable = true;

                TargetBonus = null;
            }

            if (world.TickIndex > 0 && world.TickIndex % 2500 == 1000)
            {
                foreach (var bonus in Bonuses)
                    bonus.Avaliable = false;

                TargetBonus = null;
            }
        }
    }

    public class QBonus
    {
        public VectorD Origin { get; }
        public bool Avaliable { get; set; } = false;

        public QBonus(VectorD origin)
        {
            Origin = origin;
        }
    }

    #region Physics
    public class Physic
    {
        Wizard self; World world; Game game; Move move;
        private CachedResources cached;

        // Word model
        // Clusters
        public readonly int ClusterGridWidth = 100;
        public int ClusterGridCount { get; private set; }

        private List<CircularUnit>[,] clustersGlobals;
        private List<CircularUnit>[,] clustersLocals;

        public static readonly Vector[] neighboursVectors = {new Vector(-1, -1), new Vector(0, -1), new Vector(1, -1), new Vector(-1, 0), new Vector(1, 0), new Vector(-1, 1), new Vector(0, 1), new Vector(1,1) };
        public static readonly Vector[] neisAndMeClusters = neighboursVectors.Union(new[] {new Vector(0, 0)}).ToArray();

        public static readonly int[] NextGlobalPathNodeIndexes = { 2, 1, 3, 4, 5 };

        private Building[] basesBuildings;

        // Cells
        // {1,2,4,5,8,10,16,20,25,32,40,50,80,100,125,160,200,250,400,500,800,1000,2000,4000}
        public readonly int GlobalCellGridWidth = 100;
        public int GlobalCellGridCount { get; private set; }

        public readonly int LocalCellGridWidth = 10;
        public int LocalCellGridCount { get; private set; }

        private double wizardRadius, wizardRadiusSq;

        public CellStatus[,] cellsGlobal;
        public CellStatus[,] cellsLocal;

        private static QSavedPaths dirtyHackPathsLocal = new QSavedPaths(40);
        private static QSavedPaths dirtyHackPathsGlobal = new QSavedPaths(40);
        private static VectorD lastLocalEndpoint = null;

        public Physic(Wizard self, World world, Game game, Move move, CachedResources cached)
        {
            this.self = self;
            this.world = world;
            this.game = game;
            this.move = move;
            this.cached = cached;

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

            // Globals
            clustersGlobals = new List<CircularUnit>[ClusterGridCount, ClusterGridCount];
            for (int i = 0; i < ClusterGridCount; i++)
            {
                for (int j = 0; j < ClusterGridCount; j++)
                {
                    clustersGlobals[i, j] = new List<CircularUnit>();
                }
            }

            // Fill
            AddUnitsToClustersGlobal(cached.Trees);

            // Locals
            clustersLocals = new List<CircularUnit>[ClusterGridCount, ClusterGridCount];
            for (int i = 0; i < ClusterGridCount; i++)
            {
                for (int j = 0; j < ClusterGridCount; j++)
                {
                    clustersLocals[i, j] = new List<CircularUnit>();
                }
            }

            // Fill
            AddUnitsToClustersLocal(cached.Buildings);
            AddUnitsToClustersLocal(cached.Minions);
            AddUnitsToClustersLocal(cached.Trees);
            AddUnitsToClustersLocal(cached.Wizards.Where(wizard => !wizard.IsMe));

            basesBuildings = cached.Buildings.Where(b => b.Type == BuildingType.FactionBase).ToArray();
        }

        private void GenerateCells()
        {
            // Global
            GlobalCellGridCount = (int)Math.Ceiling(game.MapSize / GlobalCellGridWidth);

            cellsGlobal = new CellStatus[GlobalCellGridCount,GlobalCellGridCount];

            // Local
            LocalCellGridCount = (int)Math.Ceiling(game.MapSize / LocalCellGridWidth);
            cellsLocal = new CellStatus[LocalCellGridCount, LocalCellGridCount];
        }

        private void AddUnitsToClustersGlobal(IEnumerable<CircularUnit> units)
        {
            foreach (var unit in units)
            {
                var p = GetCluster(unit);
                clustersGlobals[p.X, p.Y].Add(unit);
            }
        }

        private void AddUnitsToClustersLocal(IEnumerable<CircularUnit> units)
        {
            foreach (var unit in units)
            {
                var p = GetCluster(unit);
                clustersLocals[p.X, p.Y].Add(unit);
            }
        }

        public Path<QNode> FindPathTo(VectorD end)
        {
            return FindPath(new VectorD(self), end);
        }

        public Path<QNode> FindPath(VectorD start, VectorD end)
        {
            var globalDirection = end - start;
            
            VectorD nextLocalEndpoint = end;

            if (globalDirection.Length > 100)
            {
                var startNodeGlobal = GetNodeGlobal(GetCellGlobal(start));
                var endNodeGlobal = GetNodeGlobal(GetCellGlobal(end));

                Path<QNode> shortestPathGlobal = AStar.FindPath(startNodeGlobal, endNodeGlobal,
                    (node1, node2) => (node2.Vec - node1.Vec).Length,
                    node => (endNodeGlobal.Vec - node.Vec).Length);

                // Hack: TODO:
                shortestPathGlobal = dirtyHackPathsGlobal.AnalyzePath(GetCellGlobal(end), shortestPathGlobal);

                if (shortestPathGlobal == null)
                {
                    return null;
                }

                var glPath = shortestPathGlobal.Reverse().ToArray();

#if QDEBUG
            foreach (var ppp in glPath)
            {
                var pp = GetPointFromCellGlobal(ppp.LastStep.Vec);
                MyStrategy.vc.FillCircle(pp.X, pp.Y, 4.0f, 1, 0, 1);
            }
#endif

                if (glPath.Length >= 2)
                {
                    foreach (var id in NextGlobalPathNodeIndexes)
                    {
                        if (id < glPath.Length)
                        {
                            nextLocalEndpoint = GetPointFromCellGlobal(glPath[id].LastStep.Vec);

                            if (IsFreeCellLocal(nextLocalEndpoint))
                                break;
                        }
                    }
                }
            }

            if (!IsFreeCellLocal(nextLocalEndpoint))
                return null;

            if (nextLocalEndpoint != lastLocalEndpoint)
            {
                dirtyHackPathsLocal.Flush();
                lastLocalEndpoint = nextLocalEndpoint;
            }
            
            var startNodeLocal = GetNodeLocal(GetCellLocal(start));
            var endNodeLocal = GetNodeLocal(GetCellLocal(nextLocalEndpoint));

            var shortestPathLocal = AStar.FindPath(startNodeLocal, endNodeLocal,
                (node1, node2) => (node2.Vec - node1.Vec).Length,
                node => (endNodeLocal.Vec - node.Vec).Length);

            shortestPathLocal = dirtyHackPathsLocal.AnalyzePath(GetCellLocal(nextLocalEndpoint), shortestPathLocal);

            return shortestPathLocal;

            //return FindPath(GetCellGlobal(start), GetCellGlobal(end));
        }

        //public Path<QNode> FindPath(Vector start, Vector end)
        //{
        //    var startNode = GetNode(start);
        //    var endNode = GetNode(end);

        //    Path<QNode> shortestPath = AStar.FindPath(startNode, endNode, 
        //        (node1, node2) => (node2.Vec - node1.Vec).Length, 
        //        node => (endNode.Vec - node.Vec).Length);

        //    return shortestPath;
        //}

        public QNode GetNodeGlobal(Vector cell)
        {
            return new QNodeGlobal(cell, null);
        }

        public QNode GetNodeLocal(Vector cell)
        {
            return new QNodeLocal(cell, null);
        }

        public bool IsFreeCellGlobal(VectorD coord)
        {
            return IsFreeCellGlobal(GetCellGlobal(coord));
        }

        public bool IsFreeCellGlobal(Vector cell)
        {
            if (cellsGlobal[cell.X, cell.Y] == CellStatus.Unknown)
            {
                cellsGlobal[cell.X, cell.Y] = CalculateCellStatusGlobal(cell);
            }

            return cellsGlobal[cell.X, cell.Y] == CellStatus.Free;
        }

        public CellStatus CalculateCellStatusGlobal(Vector cell)
        {
            var coords = GetPointFromCellGlobal(cell);

            var centralCluster = GetCluster(coords);

            foreach (var neigh in neisAndMeClusters)
            {
                var cl = (centralCluster + neigh);

                if (cl.X < 0 || cl.Y < 0 || cl.X >= ClusterGridCount || cl.Y >= ClusterGridCount)
                    continue;

                foreach (var unit in clustersGlobals[cl.X, cl.Y])
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

        public bool IsFreeCellLocal(VectorD coord)
        {
            return IsFreeCellLocal(GetCellLocal(coord));
        }

        public bool IsFreeCellLocal(Vector cell)
        {
            if (cellsLocal[cell.X, cell.Y] == CellStatus.Unknown)
            {
                cellsLocal[cell.X, cell.Y] = CalculateCellStatusLocal(cell);
            }

            return cellsLocal[cell.X, cell.Y] == CellStatus.Free;
        }

        public CellStatus CalculateCellStatusLocal(Vector cell)
        {
            var coords = GetPointFromCellLocal(cell);

            var centralCluster = GetCluster(coords);

            foreach (var neigh in neisAndMeClusters)
            {
                var cl = (centralCluster + neigh);

                if (cl.X < 0 || cl.Y < 0 || cl.X >= ClusterGridCount || cl.Y >= ClusterGridCount)
                    continue;

                foreach (var unit in clustersLocals[cl.X, cl.Y])
                {
                    var unitPointRadius = (wizardRadius + unit.Radius + 4.0);
                    if (SquareDistance(unit, coords) <= unitPointRadius * unitPointRadius)
                    {
                        return CellStatus.Busy;
                    }
                }

                foreach (var building in basesBuildings)
                {
                    var unitPointRadius = (wizardRadius + building.Radius + 4.0);
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

        public Vector GetCellGlobal(Unit unit)
        {
            return new Vector((int)Math.Floor(unit.X / GlobalCellGridWidth), (int)Math.Floor(unit.Y / GlobalCellGridWidth));
        }

        public Vector GetCellGlobal(VectorD coord)
        {
            return new Vector((int)Math.Floor(coord.X / GlobalCellGridWidth), (int)Math.Floor(coord.Y / GlobalCellGridWidth));
        }
        
        public Vector GetCellLocal(Unit unit)
        {
            return new Vector((int)Math.Floor(unit.X / LocalCellGridWidth), (int)Math.Floor(unit.Y / LocalCellGridWidth));
        }

        public Vector GetCellLocal(VectorD coord)
        {
            return new Vector((int)Math.Floor(coord.X / LocalCellGridWidth), (int)Math.Floor(coord.Y / LocalCellGridWidth));
        }

        public VectorD GetPointFromCellGlobal(Vector cell)
        {
            return new VectorD(cell.X * GlobalCellGridWidth + GlobalCellGridWidth / 2, cell.Y * GlobalCellGridWidth + GlobalCellGridWidth / 2);
        }

        public VectorD GetPointFromCellLocal(Vector cell)
        {
            return new VectorD(cell.X * LocalCellGridWidth + LocalCellGridWidth / 2, cell.Y * LocalCellGridWidth + LocalCellGridWidth / 2);
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

    public class QSavedPaths
    {
        public int ForcedPathsCount { get; set; }
        public Dictionary<Vector, SavedPath> SavedPaths = new Dictionary<Vector, SavedPath>();

        public QSavedPaths(int forcedPathsCount)
        {
            ForcedPathsCount = forcedPathsCount;
        }

        public Path<QNode> AnalyzePath(Vector end, Path<QNode> path)
        {
            if (SavedPaths.ContainsKey(end))
            {
                return SavedPaths[end].AnalyzePath(path);
            }
            else
            {
                SavedPaths.Add(end, new SavedPath(path, ForcedPathsCount));
                return path;
            }
        }

        public void Flush()
        {
            SavedPaths.Clear();
        }

        public class SavedPath
        {
            private int forcedPaths = 0;
            private PathState state = PathState.Normal;
            private Queue<Path<QNode>> paths = new Queue<Path<QNode>>();
            public int ForcedPathsCount { get; set; }

            public SavedPath(Path<QNode> path, int forcedPathsCount)
            {
                ForcedPathsCount = forcedPathsCount;

                paths.Enqueue(path);
            }

            public Path<QNode> AnalyzePath(Path<QNode> path)
            {
                if (path == null)
                    return null; 

                var currentCellVector = path.Last().LastStep.Vec;

                switch (state)
                {
                case PathState.Normal:
                {
                    var pa = paths.Reverse().ToArray(); // Reverse queue to stack

                    if (pa.Length >= 3)
                    {
                        var steps = pa.Take(3)
                            .Select(p => p.First().PreviousSteps?.LastStep.Vec)
                            .Concat(new[] {path.First().PreviousSteps?.LastStep.Vec})
                            .ToArray();

                        var second = pa[0];

                        if (steps[0] != steps[1] && steps[0] == steps[2] && steps[1] == steps[3]) // Check parity
                        {
                            state = PathState.Forced;
                            forcedPaths = ForcedPathsCount;
                            //lastForcedCell = second.Last().LastStep.Vec;

                            return second;
                        }

                        paths.Dequeue();
                    }

                    paths.Enqueue(path);

                    break;
                }
                case PathState.Forced:
                {
                    if (--forcedPaths == 0)
                    {
                        state = PathState.Normal;

                        paths.Clear();

                        return AnalyzePath(path);
                    }
                        
                    return paths.Peek();
                }
                }

                return path;
            }

            public enum PathState
            {
                Normal = 0,
                Forced
            }
        }
    }

    public class QCluster<TNode> where TNode : IHasNeighbours<TNode>
    {
        public List<TNode> Nodes { get; set; } = new List<TNode>();
    }

    public class QNodeLocal : QNode
    {
        public QNodeLocal(Vector vec, IEnumerable<QNode> neighbours)
            : base(vec, neighbours)
        {
        }

        public override IEnumerable<QNode> CalculateNeighbours()
        {
            var Ph = MyStrategy.physic;
            var centralCell = Vec;

            List<QNode> neighbours = new List<QNode>();

            foreach (var neighbour in Physic.neighboursVectors)
            {
                var considerCell = (centralCell + neighbour);

                if (considerCell.X < 0 || considerCell.Y < 0 || considerCell.X >= Ph.LocalCellGridCount || considerCell.Y >= Ph.LocalCellGridCount)
                    continue;

                if (Ph.IsFreeCellLocal(considerCell))
                {
                    neighbours.Add(new QNodeLocal(considerCell, null));
                }
            }

            return neighbours;
        }
    }

    public class QNodeGlobal : QNode
    {
        public QNodeGlobal(Vector vec, IEnumerable<QNode> neighbours) 
            : base(vec, neighbours)
        {
        }

        public override IEnumerable<QNode> CalculateNeighbours()
        {
            var Ph = MyStrategy.physic;
            var centralCell = Vec;

            List<QNode> neighbours = new List<QNode>();

            foreach (var neighbour in Physic.neighboursVectors)
            {
                var considerCell = (centralCell + neighbour);

                if (considerCell.X < 0 || considerCell.Y < 0 || considerCell.X >= Ph.GlobalCellGridCount || considerCell.Y >= Ph.GlobalCellGridCount)
                    continue;

                if (Ph.IsFreeCellGlobal(considerCell))
                {
                    neighbours.Add(new QNodeGlobal(considerCell, null));
                }
            }

            return neighbours;
        }
    }

    public abstract class QNode : IHasNeighbours<QNode>, IEquatable<QNode>
    {
        //public Func<QNode, IEnumerable<QNode>> CalculateNeighbours { get; }
        //public Physic Ph { get; }
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
            protected set { _neighbours = value; }
        }

        public QNode(Vector vec, IEnumerable<QNode> neighbours)
        {
            Vec = vec;
            //CalculateNeighbours = calculateNeighbours;
            //Ph = physic;
            Neighbours = neighbours;
        }

        public abstract IEnumerable<QNode> CalculateNeighbours();

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
    #endregion

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
            // will throw if there isn�t any first element!
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

    #region DEBUG
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
    #endregion

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

        public static TSource Max<TSource, TComparer>(this IEnumerable<TSource> source, Func<TSource, TComparer> mappingFunc)
        {
            Comparer<TComparer> comparer = Comparer<TComparer>.Default;

            TSource value = default(TSource);
            TComparer mvalue = default(TComparer);

            if (value == null)
            {
                foreach (TSource x in source)
                {
                    var m = mappingFunc(x);

                    if (x != null && m != null && (value == null || comparer.Compare(m, mvalue) > 0))
                    {
                        value = x;
                        mvalue = m;
                    }
                }
                return value;
            }
            else
            {
                bool hasValue = false;
                foreach (TSource x in source)
                {
                    var m = mappingFunc(x);

                    if (hasValue)
                    {
                        if (comparer.Compare(m, mvalue) > 0)
                        {
                            value = x;
                            mvalue = m;
                        }
                    }
                    else
                    {
                        value = x;
                        mvalue = m;
                        hasValue = true;
                    }
                }
                if (hasValue) return value;

                throw new Exception("No elements.");
            }
        }

        public static TSource Min<TSource, TComparer>(this IEnumerable<TSource> source, Func<TSource, TComparer> mappingFunc)
        {
            Comparer<TComparer> comparer = Comparer<TComparer>.Default;

            TSource value = default(TSource);
            TComparer mvalue = default(TComparer);

            if (value == null)
            {
                foreach (TSource x in source)
                {
                    var m = mappingFunc(x);

                    if (x != null && m != null && (value == null || comparer.Compare(m, mvalue) < 0))
                    {
                        value = x;
                        mvalue = m;
                    }
                }
                return value;
            }
            else
            {
                bool hasValue = false;
                foreach (TSource x in source)
                {
                    var m = mappingFunc(x);

                    if (hasValue)
                    {
                        if (comparer.Compare(m, mvalue) < 0)
                        {
                            value = x;
                            mvalue = m;
                        }
                    }
                    else
                    {
                        value = x;
                        mvalue = m;
                        hasValue = true;
                    }
                }
                if (hasValue) return value;

                throw new Exception("No elements.");
            }
        }
    }
}