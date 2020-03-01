import java.util.*;
import java.io.*;
import java.math.*;

    class Data {
        static final int POPULATION_SIZE = 60;
        static final double BEST_POP_PART = 0.3;
        static final double RANDOM_POP_PART = 0.2;
        static final int BEST_POP_SIZE = (int) (POPULATION_SIZE * BEST_POP_PART);
        static final int RANDOM_POP_SIZE = (int) (POPULATION_SIZE * RANDOM_POP_PART);
        static final int DEPTH_SIMUL = 6;
        static final long TIMER = 74;
        static final double MUTATION_AMPLITUDE = 0.3;
    }

    class Point {
        
        double x;
        double y;
        
        Point(){
            this.x = 0; this.y = 0;
        }
        
        Point( double x, double y ){
            this.x = x;
            this.y = y;
        }
        
        double distance( Point p ){
            return Math.sqrt( distance2( p ));
        }
        
        double distance2( Point p ){
            return (this.x - p.x ) * ( this.x - p.x ) + ( this.y - p.y ) * ( this.y - p.y );
        }
        
        Point closest( Point a, Point b){
            double da = b.y - a.y;
            double db = a.x - b.x;
            
            double c1 = da * a.x + db * a.y;
            double c2 = -db * this.x + da * this.y;
            
            double det = da * da + db * db;
            
            double cx = 0.0;
            double cy = 0.0;
            
            if (det != 0.0 ) 
            {
                cx = (da * c1 - db * c2) / det;
                cy = (da * c2 + db * c1) / det;
            }
            else
            {
                cx = this.x;
                cy = this.y;
            }
            return new Point( cx, cy );
        }
    }
//====================================================    
    
    
    class Unit extends Point {
        
        int id;
        double r;
        double vx;
        double vy;
        
        Unit(int r ){
            super();
            this.r = r;
        }
        
        Unit( double x, double y ){
            super(x, y);
            this.r = 600;
        }
        
        Collision collision( Unit u ){
            double dist = this.distance2( u );
            double sr = 0;
            if (u instanceof Checkpoint)
            {
                sr = (u.r * u.r);
                if (dist < sr){
                    return new Collision( this, u, 0.0 );
                }else
                {
                    return null;
                }
            }
            else
            {
                sr = (this.r + u.r ) * ( this.r + u.r );
            }
            
            if ( dist <= sr ) 
            {
                return new Collision( this, u, 0.0 );
            }
            if ( this.vx == u.vx && this.vy == u.vy )
            {
                return null;
            }
            double x = this.x - u.x;
            double y = this.y - u.y;
            
            Point myp = new Point( x, y );
            double vx = this.vx - u.vx;
            double vy = this.vy - u.vy;
            Point up = new Point( 0, 0 );
            
            Point p = up.closest( myp, new Point( x + vx, y + vy ));
            double pdist = up.distance2( p );
            double mypDist = myp.distance2( p );
            
            if (pdist < sr )
            {
                double length = Math.sqrt( vx * vx + vy * vy );
                
                double backDist = Math.sqrt( sr - pdist );
                
                p.x = p.x - backDist * ( vx / length );
                p.y = p.y - backDist * ( vy / length );
                
                if ( myp.distance2( p ) > mypDist )
                {
                    return null;
                }
                
                pdist = p.distance2( myp );
                
                if ( pdist > length )
                {
                    return null;
                }
                
                double t = pdist / length;
                return new Collision( this, u , t );
            }
            return null;
        }
        
        void bounce( Unit u ){
            if ( u instanceof Checkpoint )
            {
                this.bounceWithCheckpoint( (Checkpoint) u );
            }
            else
            {
                Pod p1 = (Pod) this;
                Pod p2 = (Pod) u;
                
                double m1 = p1.shield ? 10 : 1;
                double m2 = p2.shield ? 10 : 1;
                
                double mcoef = (m1 + m2) / (m1 * m2);
                
                double nx = this.x - u.x;
                double ny = this.y - u.y;
                
                double nxnysquare = nx * nx + ny * ny;
                
                double dvx = this.vx - u.vx;
                double dvy = this.vy - u.vy;
                
                double product = nx * dvx + ny * dvy;
                double fx = (nx * product) / (nxnysquare * mcoef);
                double fy = (ny * product) / (nxnysquare * mcoef);
                
                this.vx -= fx / m1;
                this.vy -= fy / m1;
                u.vx += fx / m2;
                u.vy += fy / m2;
                
                double impulse = Math.sqrt(fx * fx + fy * fy);
                if (impulse < 120.0 )
                {
                    fx = fx * 120 / impulse;
                    fy = fy * 120 / impulse;
                }
                this.vx -= fx / m1;
                this.vy -= fx / m1;
                
                u.vx += fx / m2;
                u.vy += fx / m2;
            }
        }
        
        void bounceWithCheckpoint( Unit c ){
        }
    }
//=============================================    
    
    
    class Checkpoint extends Unit {
        
        Checkpoint( int x, int y){
            super(x, y);
        }
        
        void bounce( Unit u ){
            System.err.println("error bounce in class checkpoint");
        }
    }
//=============================================    
    
    
    class Pod extends Unit implements Cloneable {
        
        double angle;
        int nextCpId = 1;
        int checked = 0;
        int timeOut = 100;
        int checkpointCount;
        boolean shield, turn1 = true;
        int shieldTimer;
        
        Pod clone;
        
        Pod(){
            super(400);
            this.clone = null;
        }
        
        Pod( int checkpointCount ){
            super( 400 );
            this.checkpointCount = checkpointCount;
            this.clone = new Pod();
        }
        
        void setNextCpId( int nextCpId ){
            if (this.nextCpId != nextCpId){
                this.checked++;
                this.nextCpId = nextCpId;
            }
        }
        
        int getNextCpId(){
            if ((this.nextCpId + 1) >= this.checkpointCount) return 0;
            return (this.nextCpId + 1);
        }
        
        void copy(){
            this.clone.x = this.x;
            this.clone.y = this.y;
            this.clone.vx = this.vx;
            this.clone.vy = this.vy;
            this.clone.angle = this.angle;
            this.clone.shield = this.shield;
            this.clone.nextCpId = this.nextCpId;
            this.clone.shieldTimer = this.shieldTimer;
            this.clone.timeOut = this.timeOut;
            this.clone.checked = this.checked;
        }
        
        void reload() {
            this.x = clone.x;
            this.y = clone.y;
            this.vx = clone.vx;
            this.vy = clone.vy;
            this.angle = clone.angle;
            this.shield = clone.shield;
            this.nextCpId = clone.nextCpId;
            this.shieldTimer = clone.shieldTimer;
            this.timeOut = clone.timeOut;
            this.checked = clone.checked;
        }
        
        void activateShield(){
            this.shield = true;
            this.shieldTimer = 3;
        }
        
        double scoreRacer( Map map ){
            if (this.checked >= (this.checkpointCount * map.laps)){
                return Double.MAX_VALUE;
            }
            return this.checked * 50000 - this.distance( map.getCheckpointById( nextCpId ));
        }
        
        double scoreBloquer( Map map, Pod racerEnemy ){
            return -(1.5 * this.distance2( racerEnemy ) + this.diffAngle( racerEnemy ) + 
            1.5 * racerEnemy.diffAngle(this) + 
            this.distance(map.getCheckpointById(racerEnemy.getNextCpId())));
        }
        
        void rotate( Point p ){
            double a = this.diffAngle( p );
            this.rotate( a );
        }
        
        void rotate( double a ){
            if ( a > 18.0 ) a = 18.0;
            else if ( a < -18.0 ) a = -18.0;
            
            this.angle += a;
            
            if ( this.angle >= 360.0){
                this.angle -= 360.0;
            }
            else if ( this.angle < 0.0 ){
                this.angle += 360.0;
            }
        }
        
        double diffAngle( Point p ){
            double a = this.getAngle( p );
            
            double right = this.angle <= a ? a - this.angle : 360.0 - this.angle + a;
            double left = this.angle >= a ? this.angle - a : this.angle + 360.0 - a;
            
            if ( right < left ){
                return right;
            }
            else
                return -left;
        }
        
        double getAngle( Point p ){
            double d = this.distance( p );
            double dx = (p.x - this.x) / d;
            double dy = (p.y - this.y) / d;
            
            double a = Math.acos( dx ) * 180 / Math.PI;
            
            if (dy < 0)
            {
                a = 360.0 - a;
            }
            return a;
        }
        
        void boost( int thrust ){
            if ( this.shield ){
                this.shieldTimer--;
                if( this.shieldTimer == 0) this.shield = false;
                return;
            }
            
            double ra = this.angle * Math.PI / 180;
            
            this.vx += Math.cos( ra ) * thrust;
            this.vy += Math.sin( ra ) * thrust;
            
        }
        
        void move( double t ){
            this.x += this.vx * t;
            this.y += this.vy * t;
        }
        
        void end(){
            this.x = Math.round( this.x );
            this.y = Math.round ( this.y );
            
            this.vx = (int) (this.vx * 0.85);
            this.vy = (int) (this.vy * 0.85);
            
            timeOut--;
        }
        /*
        void play( Point p, int thrust ){
            this.rotate( p );
            this.boost( thrust );
            this.move( 1.0 );
            this.end();
        }
        */
        void play( Move m ){
            this.rotate( m.angle );
            this.boost( m.thrust );
        }
        
        void bounceWithCheckpoint (Unit u ){
            this.setNextCpId( this.getNextCpId() );
            timeOut = 100;
        }
        
        void apply( Move m ){
            if (m.shield){
                this.activateShield();
            }
            this.play( m );
        }
        
        Point getPoint( double angle ) {
            double a = this.angle + angle;
            
            if ( a >= 360.0 )
            {
                a -= 360.0;
            }
            else if ( a < 0.0 )
            {
                a += 360.0;
            }
            
            a = a * Math.PI / 180;
            double px = Math.round( this.x + Math.cos( a ) * 10000.0 );
            double py = Math.round( this.y + Math.sin( a ) * 10000.0 );
            return new Point( px, py );
        }
       /* 
        public String toString(){
            StringBuffer buff = new StringBuffer();
            buff.append("x : " + this.x + " y : " + this.y + "\n");
            buff.append("angle : " + this.angle + "\n");
            buff.append("vx : " + this.vx + " vy : " + this.vy + "\n");
            buff.append("checked : " + this.checked + "\n");
            return buff.toString();
        }
        
        public void printerr(){
            System.err.println(this);
        }
        */
        void output( Move move, Map map ){
            double a = angle + move.angle;
            
            if ( a >= 360.0 )
            {
                a = a - 360.0;
            }
            else if ( a < 0.0 )
            {
                a += 360.0;
            }
            
            a = a * Math.PI / 180;
            double px = this.x + Math.cos( a ) * 10000.0;
            double py = this.y + Math.sin( a ) * 10000.0;
            
            if ( move.shield )
            {
                System.out.println( Math.round( px ) + " " + Math.round( py ) + " SHIELD" );
                activateShield();
            }
            else if ( turn1 )
            {
                System.out.println( Math.round( map.getCheckpointById(this.nextCpId).x ) + " " + Math.round( map.getCheckpointById( this.nextCpId ).y) + " BOOST" );
                turn1 = false;
            }
            else{
                System.out.println( Math.round( px ) + " " + Math.round( py ) + " " + move.thrust );
            }
        }
    }
//==============================================    
    
    
    class Collision {
        
        Unit a, b;
        double t;
        
        Collision(){
        }
        
        Collision( Unit a, Unit b, double t ){
            this.a = a; this.b = b;
            this.t = t;
        }
        
        public boolean isSame( Collision col ){
            if ((this.a == col.a || this.a == col.b) && (this.b == col.a || this.b == col.b))
                return true;
            return false;
        }
    }
//==============================================    
    
    
    class Move implements Cloneable {
        boolean shield;
        double angle;
        int thrust;
        final static int SHIELD_PROB = 101;
        
        Move(double angle, int thrust ){
            this.angle = angle;
            this.thrust = thrust;
        }
        
        Move( double angle, int thrust, int shieldProb ){
            this(angle, thrust);
            if (shieldProb > SHIELD_PROB) this.shield = true;
            else this.shield = false;
        }
        
        Move( Move move ){
            this.shield = move.shield;
            this.angle = move.angle;
            this.thrust = move.thrust;
        }
        
        @Override
        public Object clone(){
            try {
                Move clone = (Move) super.clone();
                
                return clone;
            }catch ( CloneNotSupportedException cnse ) {
                return null;
            }
        }
        
        void mutate( double amplitude ){
            double ramin = this.angle - 36.0 * amplitude;
            double ramax = this.angle + 36.0 * amplitude;
            
            if (ramin < -18.0)
            {
                ramin = -18.0;
            }
            if (ramax > 18.0)
            {
                ramax = 18.0;
            }
            
            angle = Randomizer.random((int) ramin, (int) (Math.abs(ramax) + Math.abs(ramin)));
            
            if (!this.shield && Randomizer.random(0, 100) > SHIELD_PROB)
            {
                this.shield = true;
            }
            else
            {
                int pmin = (int) (this.thrust - 100 * amplitude);
                int pmax = (int) (this.thrust + 100 * amplitude);
                
                if (pmin < 0)
                {
                    pmin = 0;
                }
                if (pmax > 100)
                {
                    pmax = 100;
                }
                this.thrust = (int) Randomizer.random(pmin, pmax);
                if (thrust < 0) thrust = 0;
                else if (thrust > 100) thrust = 100;
                this.shield = false;
            }
        }
    }
//==================================================   
    
    
    class Solution implements Cloneable{
        Move[] moves1;
        Move[] moves2;
        boolean scoreCalculate = false;
        double score = 0.0;
        
        Solution( Move[] moves1, Move[] moves2 ){
            this.moves1 = moves1; this.moves2 = moves2;
        }
        /*
        Solution( Solution other ){
            this.moves1 = other.moves1;
            this.moves2 = other.moves2;
        }*/
        
        Move getNextMove1(){
            return moves1[0];
        }
        
        Move getNextMove2(){
            return moves2[0];
        }
        
        Move getMove1( int index ){
            return moves1[index];
        }
        
        Move getMove2( int index ){
            return moves2[index];
        }
        
        @Override
        public boolean equals( Object o ){
            if (!(o instanceof Solution)) {
                if (this != o) return true;
            }
            return false;
        }
        
        void mutate( double amplitude ){
            if( Math.random() <= 1.0){
                for (int i = 0; i < moves1.length; i++){
                    int jump = (int) Randomizer.random(1, 10);
                    if (jump == 5)
                        moves1[(int) Randomizer.random(0, moves1.length -1)].mutate(amplitude);
                }
            
                for (int i = 0; i < moves2.length; i++){
                    int jump = (int) Randomizer.random(1, 10);
                    if (jump == 5)
                    moves2[(int) Randomizer.random(0, moves2.length -1)].mutate(amplitude);
                }
            }
        }
        
        Solution[] crossOver( Solution partner ) {
            Move[] moves1a = new Move[this.moves1.length];
            Move[] moves2a = new Move[this.moves2.length];
            Move[] moves1b = new Move[this.moves1.length];
            Move[] moves2b = new Move[this.moves2.length];
            
            //int mid = (int) Randomizer.random( 1, moves1.length - 2 );
            int mid = Data.DEPTH_SIMUL / 2;
            
            for (int i = 0; i < this.moves1.length; i++){
                if (i < mid){
                    moves1a[i] = new Move(this.moves1[i]);
                    moves2a[i] = new Move(this.moves2[i]);
                    
                    moves1b[i] = new Move(partner.moves1[i]);
                    moves2b[i] = new Move(partner.moves2[i]);
                }else{
                    moves1a[i] = new Move(partner.moves1[i]);
                    moves2a[i] = new Move(partner.moves2[i]);
                    
                    moves1b[i] = new Move(this.moves1[i]);
                    moves2b[i] = new Move(this.moves2[i]);
                }
            }
            
            return new Solution[] {new Solution( moves1a, moves2a ), new Solution( moves1b, moves2b )};
        }
        
        public double score( Pod[] pods, Map map, Solution enemySolution ){
            if ( !scoreCalculate )
            {
                for (int i = 0; i < 4; i++){
                    pods[i].copy();
                }
                
                for (int i = 0; i < moves1.length; i++)
                {
                    pods[0].apply( moves1[i] );
                    pods[1].apply( moves2[i] );
                    pods[2].apply( enemySolution.moves1[i] );
                    pods[3].apply( enemySolution.moves2[i] );
                    play(pods, map);
                }
                this.score = evaluate( pods, map );
                this.scoreCalculate = true;
                
                for (int i = 0; i < 4; i++){
                    pods[i].reload();
                }
            }
            return this.score;
        }
        
        double evaluate( Pod[] pods, Map map ){
                
            return pods[0].scoreRacer(map) + pods[1].scoreRacer(map) - pods[2].scoreRacer(map);
        }
        
        void play( Pod[] pods, Map map ){
            // Il faut conserver le temps où on en est dans le tour. Le but est d'arriver à 1.0
            double t = 0.0;
            Collision last = null;

            while (t < 1.0) {
            Collision firstCollision = null;

                // On cherche toutes les collisions qui vont arriver pendant ce tour
                for (int i = 0; i < pods.length; ++i) {
                    // Collision avec un autre pod ?
                    for (int j = i + 1; j < pods.length; ++j) {
                        Collision col = pods[i].collision(pods[j]);

                        // Si la collision arrive plus tôt que celle qu'on, alors on la garde
                        if (col != null && col.t + t < 1.0 && (firstCollision == null || col.t < firstCollision.t)) {
                            if(last != null && !last.isSame(col))
                            firstCollision = col;
                        }
                    }

                    // Collision avec un checkpoint ?
                    // Inutile de tester toutes les checkpoints ici. On test juste le prochain checkpoint du pod.
                    // On pourrait chercher les collisions du pod avec tous les checkpoints, mais si une telle collision arrive elle n'aura aucun impact sur le jeu de toutes façons
                    Collision col = pods[i].collision(map.getCheckpointById(pods[i].nextCpId));

                    // Si la collision arrive plus tôt que celle qu'on, alors on la garde
                    if (col != null && col.t + t < 1.0 && (firstCollision == null || col.t < firstCollision.t)) {
                        col.a.bounce( col.b );
                    }
                }

                if (firstCollision == null) {
                    // Aucune collision, on peut juste déplacer les pods jusqu'à la fin du tour
                    for (int i = 0; i < pods.length; ++i) {
                        pods[i].move(1.0 - t);
                    }

                    // Fin du tour
                    t = 1.0;
                } else {
                    // On bouge les pods du temps qu'il faut pour arriver sur l'instant `t` de la collision
                    for (int i = 0; i < pods.length; ++i) {
                        pods[i].move(firstCollision.t);
                    }

                    // On joue la collision
                    firstCollision.a.bounce(firstCollision.b);

                    t += firstCollision.t;
                }
            }
            
            // On arrondi les positions et on tronque les vitesses pour tout le monde
            for (int i = 0; i < pods.length; ++i) {
                pods[i].end();
            }
        }
    }
//==================================================   
    
    
    class Map {
        int laps;
        int nbrCheckpoint;
        Checkpoint[] checkpoints;
        
        Map () {
            this.checkpoints = null;
        }
        
        Map( Checkpoint[] checkpoints ){
            this.checkpoints = checkpoints;
        }
        
        Map( Checkpoint[] checkpoints, int laps, int nbrCheckpoint){
            this.checkpoints = checkpoints;
            this.laps = laps;
            this.nbrCheckpoint = nbrCheckpoint;
        }
        
        Checkpoint getCheckpointById( int id ){
            return checkpoints[id];
        }
    }
//===============================================    
    
    
    class Generator {
        int RANDOM_COUNT;
        CasteljauHelper casteljau;
        Map map;
        boolean firstTurn = true;
        
        Generator( Map map ) {
            this.map = map;
            this.casteljau = new CasteljauHelper();
            RANDOM_COUNT = Data.POPULATION_SIZE - 2;
        }
        
        public void generatePopulation( Pod[] pods, Solution lastSolution, ArrayList<Solution> solutions ) {
            Solution first = null;
            
            first = straitToPoint( pods, 100 );
            solutions.add( first );
            
            if (lastSolution != null)
            {
                reuseSolution( lastSolution );
                solutions.add( lastSolution );
                if( firstTurn )
                {
                    this.RANDOM_COUNT--;
                    this.firstTurn = false;
                }
            }
            
            first = generateSolutionByIA( pods );
            solutions.add( first );
            
           for (int i = 0; i < RANDOM_COUNT; i++){
                Solution random = generateRandom();
                solutions.add( random );
            }
        }
        
        Solution generateEnemySolution( Pod[] pods ) {
            Move[] moves1 = new Move[Data.DEPTH_SIMUL];
            Move[] moves2 = new Move[Data.DEPTH_SIMUL];
            
            moves1 = generateBezierCurve( pods[2] );
            moves2 = generateBezierCurve( pods[3] );
            
            return new Solution( moves1, moves2 );
        }
        
        void reuseSolution( Solution lastSolution ){
            
            lastSolution.scoreCalculate = false;
            
            for (int i = 1; i < Data.DEPTH_SIMUL; i++){
                lastSolution.moves1[i - 1] = lastSolution.moves1[i];
                lastSolution.moves2[i - 1] = lastSolution.moves2[i];
            }
            lastSolution.moves1[Data.DEPTH_SIMUL - 1] = generateRandomMove();
            lastSolution.moves2[Data.DEPTH_SIMUL - 1] = generateRandomMove();
        }
        
        Solution generateSolutionByIA( Pod[] pods ){
            Move[] bezierMoves = generateBezierCurve( pods[0] );
            Move[] toRacerMoves = straitToPoint( pods[1], pods[2], 50 );
            
            return new Solution( bezierMoves, toRacerMoves );
        }
        
        Move[] generateBezierCurve( Pod pod ){
            double dist = pod.distance2( map.getCheckpointById( pod.nextCpId ));
            double vel = pod.vx * pod.vx + pod.vy * pod.vy;
            double diffangle = pod.diffAngle( map.getCheckpointById( pod.getNextCpId()));
            if ( dist < ((3 * vel) / diffangle) )
            {
                Move[] curve = new Move[Data.DEPTH_SIMUL];
                List<Point> list = new ArrayList<>(3);
                list.add( pod );
                list.add( map.getCheckpointById( pod.nextCpId ));
                list.add( map.getCheckpointById( pod.getNextCpId() ));
                casteljau.createCurve( list );
            
                for (int i = 0; i < Data.DEPTH_SIMUL; i++){
                    double angle;
                    int thrust;
                    Point p = casteljau.getNextCurvePoint();
                    angle = pod.diffAngle( p );
                    thrust = (int) (dist - 4 * vel);
                    if (thrust > 100) thrust = 100;
                    else if (thrust < 0) thrust = 0;
                    curve[i] = new Move( angle, thrust );
                }
                return curve;
            }
            else
            {
                return this.straitToPoint( pod, map.getCheckpointById( pod.nextCpId ), 100 );
            }
        }
        
        Move generateRandomMove(){
            return new Move(
                        Randomizer.randomInt(-18, 18),
                        Randomizer.randomInt(0, 100),
                        Randomizer.randomInt(0, 100));
        }
        
        Move[] generateRandomSequence() {
            Move[] moves = new Move[Data.DEPTH_SIMUL];
            for (int i = 0; i < Data.DEPTH_SIMUL; i++){
                moves[i] = generateRandomMove();
            }
            return moves;
        }
        
        Solution generateRandom(){
            Move[] moves1 = generateRandomSequence();
            Move[] moves2 = generateRandomSequence();
            
            return new Solution(moves1, moves2);
        }
        
        Move[] straitToPoint( Pod pod, Point point, int thrust ){
            pod.copy();
            Move[] moves = new Move[Data.DEPTH_SIMUL];
            for (int i = 0; i < Data.DEPTH_SIMUL; i++){
                double angle = pod.diffAngle( point );
                //pod.play( cp, 100 );
                moves[i] = new Move( angle, thrust );
                pod.play( moves[i] );
                pod.move( 1.0 );
                pod.end();
            }
            pod.reload();
            
            return moves;
        }
        
        Solution straitToPoint( Pod[] pods, int thrust ){
            Move[] moves1 = this.straitToPoint( pods[0], map.getCheckpointById( pods[0].nextCpId ), thrust );
            Move[] moves2 = this.straitToPoint( pods[1], pods[2], thrust );
            
            return new Solution(moves1, moves2);
        }
    }
//=================================================   
    
    
    class Mutator {
        ArrayList<Solution> population, matingPool;
        int matingPoolSize;
        int restPop;
        
        
        Mutator(){
            this.matingPoolSize = Data.POPULATION_SIZE / 2;
            this.restPop = Data.POPULATION_SIZE - this.matingPoolSize;
        }
        
        
        Solution mutate( Pod[] pods, Map map, ArrayList<Solution> solutions, Solution enemySolution ){
            long last, sum;
            //int generation = 0;
            
            this.population = solutions;
            last = System.currentTimeMillis();
            sum = last + Data.TIMER;
            
            while ( last < sum )
            {
                this.evaluate( pods, map, enemySolution );
                
                this.selection();
                
                //if (generation == 0) System.err.println(population.get(0).score);
                //generation++;
                
                last = System.currentTimeMillis();
            }
            
            population.sort((a,b)->Double.compare(b.score(pods,map,enemySolution), a.score(pods,map,enemySolution)));
            //System.err.println(generation + " " + population.get(0).score);
            
            return population.get(0);
        }
        
        void evaluate( Pod[] pods, Map map, Solution enemySolution ) {
            this.population.sort((a,b)->Double.compare(b.score(pods,map,enemySolution), a.score(pods,map,enemySolution)));
            //double sum = this.population.stream().mapToDouble(b->b.score).sum();
            //System.err.println(sum / Data.POPULATION_SIZE + " " +population.get(0).score);
            this.matingPool = new ArrayList<>(this.matingPoolSize);
            
            for (int i = 0; i < Data.BEST_POP_SIZE; i++){
                this.matingPool.add( population.get(i) );
            }
            
            for (int i = 0; i < Data.RANDOM_POP_SIZE; i++){
                this.matingPool.add( population.get(Randomizer.randomInt(Data.BEST_POP_SIZE, Data.POPULATION_SIZE - 1)));
            }
        }
        
        void selection() {
            ArrayList<Solution> newPop = new ArrayList<>(Data.POPULATION_SIZE);
            
            
            newPop.addAll( this.matingPool );
            
            for (int i = 0, max = this.matingPoolSize / 2; i < max; i++){
                int random = i;
                
                while ( random == i )
                {
                    random = Randomizer.randomInt( 0, this.matingPoolSize - 1 );
                }
                
                Solution parentA = this.matingPool.get( i );
                Solution parentB = this.matingPool.get( random );
                
                Solution[] children = parentA.crossOver( parentB );
                children[0].mutate( Data.MUTATION_AMPLITUDE );
                children[1].mutate( Data.MUTATION_AMPLITUDE );
                newPop.add( children[0] );
                newPop.add( children[1] );
            }
            
            this.population = newPop;
        }
    }
//==========================================   
    
    
    class Simulator {
        
        Map map;
        Pod[] pods = null;
        Generator generator;
        Mutator mutator;
        Solution lastSolution = null;
        ArrayList<Solution> solutions = new ArrayList<>(Data.POPULATION_SIZE);
        boolean swap = false;
        
        
        Simulator( Map map ){
            this.map = map;
            this.generator = new Generator( map );
            this.mutator = new Mutator();
        }
        
        Solution simulate( Pod[] pods ) {
            
            affectJob( pods );
            
            Solution enemySolution = generator.generateEnemySolution( pods );
            
            solutions.clear();
            
            generator.generatePopulation( pods, lastSolution, solutions );
            
            lastSolution = mutator.mutate( pods, map, solutions, enemySolution );
            
            swap( pods );
            
            return lastSolution;
        }
        
        void affectJob( Pod[] pods ) {
            if (pods[3].checked >= pods[2].checked && 
                pods[3].distance2( map.getCheckpointById( pods[3].nextCpId )) < 
                            pods[2].distance2( map.getCheckpointById( pods[2].nextCpId )))
            {
                 Pod tmp = pods[3];
                 pods[3] = pods[2];
                 pods[2] = tmp;
                 swap = true;
            }
        }
        
        void swap ( Pod[] pods ){
            if (swap){
                Pod tmp = pods[3];
                 pods[3] = pods[2];
                 pods[2] = tmp;
                 swap = false;
            }
        }
    }
//==========================================


    class CasteljauHelper {
        private List<Point> ctrlPoints;
        private ArrayList<Point> tempPoints = new ArrayList<>(3);
	    ArrayList<Point> curvePoints;
	    private Point tmp;
	    private int current;
	    
	    public CasteljauHelper(){
	        this.curvePoints = new ArrayList<>(5);
	    }

        public void createCurve( List<Point> checkpoints ) {
            current = 0;
            curvePoints.clear();
            ctrlPoints = checkpoints;
            evalCasteljau();
        }
        
	    public void evalCasteljau(){
	        for(double i = 1; i<= Data.DEPTH_SIMUL; i++){
			    tmp = getCasteljauPoint(i / Data.DEPTH_SIMUL);
			    curvePoints.add(tmp);
		    } 
    	}

	    public Point getCasteljauPoint(double t){
		tempPoints.clear();
		int n = ctrlPoints.size()-1;

		for(int i = 0; i<=n; i++){
			Point p = new Point(ctrlPoints.get(i).x,ctrlPoints.get(i).y);
			tempPoints.add(i, p);		
		}

		for (int k = 1; k<=n;k++){
			for(int i = 0; i<=n-k; i++){
				tempPoints.get(i).x = t*tempPoints.get(i).x+(1-t)*tempPoints.get(i+1).x;
				tempPoints.get(i).y = t*tempPoints.get(i).y+(1-t)*tempPoints.get(i+1).y;
			}
		}

		return tempPoints.get(0);
	    }
	    
	    public double sqr( double d ){
	        return Math.pow( d, 2 );
	    }

    	public Point getNextCurvePoint() {
    	    current++;
    	    return curvePoints.get( current - 1 );
    	}
    }
//==========================================

    
 class Player {
    
   

    public static void main(String args[]) {
        Pod[] pods = new Pod[4];
        Map map;
        Checkpoint[] checkpoints;
        Scanner in = new Scanner(System.in);
        int laps = in.nextInt();
        int checkpointCount = in.nextInt();
        checkpoints = new Checkpoint[checkpointCount];
        for (int i = 0; i < checkpointCount; i++) {
            int checkpointX = in.nextInt();
            int checkpointY = in.nextInt();
            checkpoints[i] = new Checkpoint(checkpointX, checkpointY);
        }
        
        for (int i = 0; i < pods.length; i++){
            pods[i] = new Pod(checkpointCount);
        }
        
        map = new Map(checkpoints, laps, checkpointCount);
        Simulator simulator = new Simulator( map );

        // game loop
        while (true) {
            long start = System.currentTimeMillis();
            for (int i = 0; i < 4; i++) {
                pods[i].x = in.nextInt(); // x position of your pod
                pods[i].y = in.nextInt(); // y position of your pod
                pods[i].vx = in.nextInt(); // x speed of your pod
                pods[i].vy = in.nextInt(); // y speed of your pod
                pods[i].angle = in.nextInt(); // angle of your pod
                pods[i].setNextCpId( in.nextInt() ); // next check point id of your pod
            }
            
            Solution solution = simulator.simulate( pods );
            
            pods[0].output(solution.getNextMove1(), map);
            pods[1].output(solution.getNextMove2(), map);
        }
    }
}

    class Randomizer {
        static Random random = new Random();
        
        static int randomInt(int min, int bound){
            if (min == 0 && bound == 0) return 0;
            double res = 0;
            
            if (min > 0)
                return random.nextInt(bound - min + 1) + min; 
                
            return random.nextInt(bound + Math.abs(min) + 1) + min;
        }
        
        static double random(int min, int bound){
            return (double) randomInt(min,bound);
        }
    }
