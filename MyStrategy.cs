using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;


using Com.CodeGame.CodeWizards2016.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeWizards2016.DevKit.CSharpCgdk {
    public sealed class MyStrategy : IStrategy {
        public void Move(Wizard self, World world, Game game, Move move) {
            move.Speed = game.WizardForwardSpeed;
            move.StrafeSpeed = game.WizardStrafeSpeed;
            move.Turn = game.WizardMaxTurnAngle;
            move.Action = ActionType.MagicMissile;
        }
    }

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

        public double SqrLength => X * X + Y * Y;

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

}