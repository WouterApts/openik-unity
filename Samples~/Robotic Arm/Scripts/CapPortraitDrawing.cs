using System.Collections.Generic;
using UnityEngine;

namespace OpenIK.Showcase
{
    /// <summary>Pen paths traced from the cap-and-mustache sketch, in upright canvas coordinates.</summary>
    public static class CapPortraitDrawing
    {
        public sealed class Stroke
        {
            public readonly int ColorIndex;
            public readonly List<Vector2> Points = new List<Vector2>();
            public Stroke(int color) { ColorIndex = color; }
            public Stroke Move(float x, float y) { Points.Add(Map(x, y)); return this; }
            public Stroke Curve(float ax, float ay, float bx, float by, float x, float y)
            {
                Vector2 p = Points[Points.Count - 1], a = Map(ax, ay), b = Map(bx, by), end = Map(x, y);
                for (int i = 1; i <= 24; i++)
                {
                    float t = i / 24f, s = 1f - t;
                    Points.Add(s * s * s * p + 3f * s * s * t * a + 3f * s * t * t * b + t * t * t * end);
                }
                return this;
            }
        }

        // Color order matches the demo's red, black, blue, and gold buckets.
        public static List<Stroke> Create()
        {
            var paths = new List<Stroke>();
            // Red cap crown, sweeping brim, and badge. The M is the finishing stroke.
            paths.Add(new Stroke(0).Move(198, 551).Curve(151, 511, 187, 402, 269, 394)
                .Move(300, 395).Curve(419, 239, 525, 168, 625, 164)
                .Curve(825, 158, 897, 252, 831, 457));
            paths.Add(new Stroke(0).Move(198, 551).Curve(359, 469, 543, 387, 697, 387)
                .Curve(761, 388, 810, 423, 831, 457)
                .Curve(695, 408, 459, 460, 198, 551));
            paths.Add(new Stroke(0).Move(412, 425).Curve(365, 375, 440, 246, 552, 246)
                .Curve(665, 232, 730, 290, 712, 376));
            // Red shirt sleeves under the overalls.
            paths.Add(new Stroke(0).Move(221, 838).Curve(181, 889, 137, 963, 108, 1020));
            paths.Add(new Stroke(0).Move(812, 826).Curve(852, 888, 889, 955, 920, 1000));

            // Black sideburns and ears, two arched brows and eyes.
            paths.Add(new Stroke(1).Move(247, 536).Move(230, 626).Move(203, 615).Move(205, 551));
            paths.Add(new Stroke(1).Move(779, 477).Move(783, 560).Move(827, 563).Move(823, 478));
            paths.Add(new Stroke(1).Move(199, 555).Curve(128, 501, 92, 575, 121, 635)
                .Curve(140, 674, 175, 661, 197, 651));
            paths.Add(new Stroke(1).Move(828, 510).Curve(919, 445, 970, 568, 904, 620)
                .Curve(884, 637, 863, 637, 845, 631));
            paths.Add(new Stroke(1).Move(270, 585).Curve(270, 513, 343, 466, 388, 515)
                .Curve(400, 529, 404, 543, 403, 555).Curve(351, 518, 316, 535, 270, 585));
            paths.Add(new Stroke(1).Move(652, 548).Curve(674, 478, 731, 456, 765, 498)
                .Curve(778, 514, 777, 529, 780, 544).Curve(737, 509, 697, 508, 652, 548));
            paths.Add(Ellipse(1, 341, 609, 31, 31));
            paths.Add(Ellipse(1, 726, 588, 34, 31));
            // The nose is drawn now; the mustache is added after the clothing.
            paths.Add(new Stroke(1).Move(420, 661).Curve(431, 604, 485, 584, 553, 587)
                .Curve(627, 570, 659, 624, 638, 686).Curve(624, 744, 566, 773, 507, 764)
                .Curve(448, 767, 406, 729, 420, 661));

            // One unbroken blue path. Retrace the strap edges rather than lifting;
            // all connecting lines run through button centers and are covered in gold.
            paths.Add(new Stroke(2).Move(238, 845).Move(173, 1020)
                .Curve(387, 1037, 646, 1031, 861, 1018)
                .Move(811, 839).Move(837, 931).Move(712, 940)
                .Move(768, 899).Move(769, 817).Move(768, 899).Move(712, 940)
                .Move(640, 904).Move(637, 848).Move(640, 904).Move(712, 940)
                .Curve(553, 953, 460, 960, 313, 947)
                .Move(361, 912).Move(373, 861).Move(361, 912).Move(313, 947)
                .Move(271, 903).Move(276, 838).Move(271, 903).Move(313, 947)
                .Move(202, 944).Move(238, 845));
            // Concentric spirals let the real brush fill each golden button.
            paths.Add(Button(313, 947, 53, 57));
            paths.Add(Button(712, 940, 67, 54));

            // Finish with one continuous mustache outline, from one side of the nose
            // around the scalloped lower edge to the other side, then the red M.
            paths.Add(new Stroke(1).Move(420, 661).Curve(354, 659, 275, 650, 193, 659)
                .Curve(172, 743, 178, 837, 238, 840)
                .Curve(273, 842, 293, 827, 312, 814).Curve(380, 896, 454, 891, 512, 825)
                .Curve(577, 882, 655, 868, 708, 785).Curve(790, 838, 860, 870, 880, 751)
                .Curve(885, 724, 887, 692, 884, 673).Curve(883, 660, 882, 650, 879, 643)
                .Curve(810, 648, 731, 649, 648, 642));
            paths.Add(new Stroke(0).Move(421, 414).Move(510, 265).Move(568, 339).Move(629, 252).Move(660, 380));
            return paths;
        }

        private static Vector2 Map(float x, float y) => new Vector2(
            0.08f + (x - 100f) / 840f * 0.84f, 0.12f + (y - 160f) / 880f * 0.72f);

        private static Stroke Ellipse(int color, float x, float y, float rx, float ry)
        {
            var path = new Stroke(color);
            for (int i = 0; i <= 64; i++)
            {
                float a = i * Mathf.PI * 2f / 64f;
                path.Move(x + Mathf.Cos(a) * rx, y + Mathf.Sin(a) * ry);
            }
            return path;
        }

        private static Stroke Button(float x, float y, float rx, float ry)
        {
            Stroke path = Ellipse(3, x, y, rx, ry);
            const int turns = 7, samples = turns * 48;
            for (int i = 1; i <= samples; i++)
            {
                float t = i / (float)samples, a = t * turns * Mathf.PI * 2f;
                path.Move(x + Mathf.Cos(a) * rx * (1f - t), y + Mathf.Sin(a) * ry * (1f - t));
            }
            return path;
        }
    }
}
