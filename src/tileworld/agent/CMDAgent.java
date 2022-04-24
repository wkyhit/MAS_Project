package tileworld.agent;

import sim.util.Int2D;
import tileworld.Parameters;
import tileworld.environment.*;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.DefaultTWPlanner;

import java.util.ArrayList;
import java.util.PriorityQueue;

public class CMDAgent extends TWAgent {
    //Agent action Mode
    enum Mode {
        EXPLORE, COLLECT, FILL, REFUEL, WAIT
    }

    private double fuelThreshold = Parameters.fuelThreshold;
    private double fuelLimit = Parameters.hardFuelLimit;
    private double objectLifetimeThreshold = Parameters.objectLifetimeThreshold;

    //Agent Information
    private String name;
    private int agentId;
    private int agentNum;
    private Mode mode; //acting mode
    private boolean[] block_visited;
    private boolean isBackRoute;
    private DefaultTWPlanner routePlanner;//route planner
    private TWAgentAreaMemory agentAreaMemory;
    private Integer[] zoneBelongToAgent;
    // Zone(belong to current agent) bound coordinate: clock-wise from top-left
    private Int2D[] zone_coordinates;
    private boolean isDivideByHeight;
    private Int2D[] blocks; // divide zone into several blocks
    private int block_num;
    private int cur_block_id;//the block that agent currently locate at
    private int hor_blocks;
    private int ver_blocks;

    //get holes and tiles from memory
    PriorityQueue<TWEntity> tile_queue_zone;
    PriorityQueue<TWEntity> hole_queue_zone;
    ArrayList<TWEntity> tilesList;
    ArrayList<TWEntity> holesList;

    //store the fuel station position
    private Int2D fuelStation;


    public CMDAgent(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;
        this.fuelStation = null;
        this.isBackRoute = false;
        this.agentId = Character.getNumericValue(name.charAt(name.length() - 1)) - 1;
        this.routePlanner = new DefaultTWPlanner(this);
        this.memory = new TWAgentAreaMemory(this, env.schedule, env.getxDimension(), env.getyDimension());
        this.agentAreaMemory = (TWAgentAreaMemory) this.memory;
        this.zone_coordinates = new Int2D[4];
        this.tilesList = new ArrayList<>();
        this.holesList = new ArrayList<>();
        this.tile_queue_zone = new PriorityQueue<>();
        this.hole_queue_zone = new PriorityQueue<>();

    }

    @Override
    public void communicate() {
        //send the agent position to all(env)
        Message message = new MyMessage(this.name, "All", "Agent_pos", new Object[]{new Int2D(this.x, this.y)});
        this.getEnvironment().receiveMessage(message);
        // once find the fuel station, set it
        if (this.fuelStation == null) {
            if (agentAreaMemory.getFuelStation() != null) {//find the station by itself
                this.fuelStation = agentAreaMemory.getFuelStation();
                /**
                 * need to broadcast the fuel station position to other agents
                 */
                Message msg_fuel = new MyMessage(this.name, "All", "fuel_station", new Object[]{this.fuelStation});
                this.getEnvironment().receiveMessage(msg_fuel);
            }else{
                // see if anyone find the fuel station
                ArrayList<Message> messages = this.getEnvironment().getMessages();
                for (int i = 0; i < messages.size(); i++) {
                    MyMessage msg = (MyMessage) messages.get(i);
                    if (msg.getTo().equals("All") && msg.getMessage().equals("fuel_station")) {
                        this.fuelStation = (Int2D) msg.getDetail()[0];
                        break;
                    }
                }
            }
        }else {
            //keep broadcasting the fuel station position
            Message msg_fuel = new MyMessage(this.name, "All", "fuel_station", new Object[]{this.fuelStation});
            this.getEnvironment().receiveMessage(msg_fuel);
        }
        if (this.zone_coordinates[0] != null) {// not the init stage
            // get the tiles and holes in the zone that current agent belong to
            tile_queue_zone = agentAreaMemory.getObjectsInZone(zone_coordinates, TWTile.class);
            hole_queue_zone = agentAreaMemory.getObjectsInZone(zone_coordinates, TWHole.class);

            //need to reset
            tilesList.clear();
            holesList.clear();
            /**
             * need to do filter: according to the object lifetime left
             */
            while (!tile_queue_zone.isEmpty()) {
                TWEntity t = tile_queue_zone.poll();
                double dist = this.getDistanceTo(t);
                // add the feasible tile to list
                if (agentAreaMemory.getEstimatedRemainLife(t, objectLifetimeThreshold) > dist) {
                    tilesList.add(t);
                }//else do nothing
            }
            while (!hole_queue_zone.isEmpty()) {
                TWEntity t = hole_queue_zone.poll();
                double dist = this.getDistanceTo(t);
                // add the feasible hole to list
                if (agentAreaMemory.getEstimatedRemainLife(t, objectLifetimeThreshold) > dist) {
                    holesList.add(t);
                }
            }
        }
    }

    /**
     * In Initialization step: divide the board into zones acc to the agent number
     * Assign zones to agents acc to the relative distance
     *
     * @param origin_x
     * @param origin_y
     * @param zone_width
     * @param zone_height
     */
    private void AssignZone(int origin_x, int origin_y, int zone_width, int zone_height) {
        // calculate the total number of agents
        ArrayList<Message> messages = this.getEnvironment().getMessages();
        agentNum = 0;
        for (int i = 0; i < messages.size(); i++) {
            MyMessage message = (MyMessage) messages.get(i);
            if (message.getTo().equals("All") && message.getMessage().equals("Agent_pos")) {
                agentNum++;
            }
        }

        //divide board into several zones acc to number of agents
        Int2D[] agent_pos = new Int2D[agentNum];
        //get other agent position
        for (int i = 0; i < messages.size(); i++) {
            MyMessage message = (MyMessage) messages.get(i);
            if (message.getTo().equals("All") && message.getMessage().equals("Agent_pos")) {
                // get the corresponding position
                agent_pos[Character.getNumericValue(message.getFrom().charAt(message.getFrom().length() - 1)) - 1] = (Int2D) message.getDetail()[0];
            }
        }

        //divide by the longest side
        isDivideByHeight = true;
        if (zone_width > zone_height) {
            isDivideByHeight = false;
        }

        Int2D zonesArea;// the size of zone
        zoneBelongToAgent = new Integer[agentNum];
        boolean[] assigned = new boolean[agentNum];

        if (isDivideByHeight) {
            zonesArea = new Int2D(zone_width, zone_height / agentNum);
        } else {
            zonesArea = new Int2D(zone_width / agentNum, zone_height);
        }

        // assign zone to agent according to distance
        for (int i = 0; i < agentNum; i++) {//loop for each zone
            int[] distance = new int[agentNum];
            //calculate the distance from cur_zone to each agent
            for (int j = 0; j < agentNum; j++) {
                if (!assigned[j]) {
                    if (isDivideByHeight) {
                        distance[j] = Math.abs(agent_pos[j].y - ((zonesArea.y * i) + origin_y));
                    } else {
                        distance[j] = Math.abs(agent_pos[j].x - ((zonesArea.x * i) + origin_x));
                    }
                } else {//already assigned, mark as infinite
                    distance[j] = 100000;
                }
            }
            int agent_assign = 0;
            int min_dis = distance[0];
            //choose the closest agent
            for (int k = 1; k < agentNum; k++) {
                if (distance[k] < min_dis) {
                    agent_assign = k;
                    min_dis = distance[k];
                }
            }
            //assign zone "i" to this agent
            zoneBelongToAgent[agent_assign] = i;
            assigned[agent_assign] = true;
        }

        //calculate the four vertices of these zones
        int zone_id = zoneBelongToAgent[agentId];
        if (isDivideByHeight) {
            zone_coordinates[0] = new Int2D(origin_x, (origin_y + (zonesArea.y * zone_id)));//top left
            zone_coordinates[1] = new Int2D(origin_x + zone_width, (origin_y + (zonesArea.y * zone_id)));//top right
            if (zone_id == agentNum - 1) {//if last zone(remaining)
                zone_coordinates[2] = new Int2D(origin_x + zone_width, origin_y + zone_height);//bottom right
                zone_coordinates[3] = new Int2D(origin_x, origin_y + zone_height);//bottom left
            } else {
                zone_coordinates[2] = new Int2D(origin_x + zone_width, origin_y + (zonesArea.y * (zone_id + 1)));//bottom right
                zone_coordinates[3] = new Int2D(origin_x, origin_y + (zonesArea.y * (zone_id + 1)));//bottom right
            }
        } else {//divide by width
            zone_coordinates[0] = new Int2D(origin_x + (zonesArea.x * zone_id), origin_y);//top left
            zone_coordinates[3] = new Int2D(origin_x + (zonesArea.x * zone_id), origin_y + zone_height);//bottom left
            if (zone_id == agentNum - 1) {//if last zone
                zone_coordinates[1] = new Int2D(origin_x + zone_width, origin_y);//top right
                zone_coordinates[2] = new Int2D(origin_x + zone_width, origin_y + zone_height);//bottom right
            } else {
                zone_coordinates[1] = new Int2D(origin_x + (zonesArea.x * (zone_id + 1)), origin_y);//top right
                zone_coordinates[2] = new Int2D(origin_x + (zonesArea.x * (zone_id + 1)), origin_y + zone_height);//bottom right
            }
        }

        /**
         * divide zone into several blocks, manage the blocks in the order of "->" ,"<-" ,"->",...
         */
//        int block_size = Parameters.defaultSensorRange * 2 + 1;
        // use smaller block
        int block_size = Parameters.defaultSensorRange * 2 - 1;
        Int2D zoneDim = new Int2D(zone_coordinates[1].x - zone_coordinates[0].x, zone_coordinates[3].y - zone_coordinates[0].y);
        hor_blocks = (int) Math.ceil(zoneDim.x / block_size);
        ver_blocks = (int) Math.ceil(zoneDim.y / block_size);
        block_num = hor_blocks * ver_blocks;
        blocks = new Int2D[block_num];
        //init the block_visited, for later exploration mission
        block_visited = new boolean[block_num];
        for (int i = 0; i < block_num; i++) {
            block_visited[i] = false;
        }
        int block_id = 0;
        for (int i = 0; i < ver_blocks; i++) {//iterate by row
            Int2D[] blocks_row = new Int2D[hor_blocks];//store the blocks in each row
            for (int j = 0; j < hor_blocks; j++) {
                int x, y;
                if (i == ver_blocks - 1) {//if last row
                    y = zone_coordinates[2].y - Parameters.defaultSensorRange;
                } else {
                    y = zone_coordinates[0].y + block_size * i + Parameters.defaultSensorRange;
                }
                if (j == hor_blocks - 1) {//if last column
                    x = zone_coordinates[2].x - Parameters.defaultSensorRange;
                } else {
                    x = zone_coordinates[0].x + block_size * j + Parameters.defaultSensorRange;
                }
                blocks_row[j] = new Int2D(x, y);
            }
            // manage the blocks in the order of shape "S"
            if (i % 2 == 0) {//for even row: in normal order
                for (int k = 0; k < hor_blocks; k++) {
                    blocks[block_id] = blocks_row[k];
                    block_id++;
                }
            } else { // for odd row: in reversed order
                for (int k = hor_blocks - 1; k >= 0; k--) {
                    blocks[block_id] = blocks_row[k];
                    block_id++;
                }
            }
        }

    }

    /**
     * get the block that agent current locate at
     * @return
     */
    private int getCurBlockId() {
        int blockId = 0;
        int row = 0, col = 0;
//        int blockSize = Parameters.defaultSensorRange * 2 + 1;
        // use smaller block
        int blockSize = Parameters.defaultSensorRange * 2 - 1;

        //reformat the x,y into its own zone x',y'
        int zone_id = zoneBelongToAgent[this.agentId];
        int new_x=this.x,new_y=this.y;
        // calculate the relative x,y in the corresponding zone
        if(this.isDivideByHeight){
            int zone_height = this.getEnvironment().getyDimension()/this.agentNum;
            new_y -= zone_id*zone_height;
        }else{
            int zone_width = this.getEnvironment().getxDimension()/this.agentNum;
            new_x -= zone_id*zone_width;
        }
        row = (int) Math.ceil(new_y/ blockSize);
        col = (int) Math.ceil(new_x / blockSize);
        //special process, if col or row >= hor_blocks or ver_blocks, it is out of scope
        // need to set it as the last row or last column
        if (row >= ver_blocks){
            row = ver_blocks - 1;
        }
        if (col >= hor_blocks){
            col = ver_blocks - 1;
        }
        if (row % 2 == 0) {//even row: normal order
            blockId = row * hor_blocks + col;
        } else {//odd row: reversed order
            blockId = row * hor_blocks + (hor_blocks - col - 1);
        }
        return blockId;
    }

    @Override
    protected TWThought think() {
        /**
         * Assign Zones to Agents in the first time
         */
        if (this.zone_coordinates[0] == null) {
            this.AssignZone(0, 0, Parameters.xDimension, Parameters.yDimension);
        }

        /**
         * default: explore(except backtrace)
         */
        if (mode != Mode.REFUEL){
            mode = Mode.EXPLORE;
        }

        /**
         * priority: 1- find fuel or refuel
         * 2-ensure carried-tiles>0
         * 3-fill hole
         */
        if (this.fuelStation==null && this.fuelLevel<this.fuelLimit && agentAreaMemory.isZoneAllVisited(zone_coordinates[0].x,zone_coordinates[1].x,zone_coordinates[0].y,zone_coordinates[3].y)){
            // (reach the fuel limit && already completely scan the own zone) and have not found station, need to wait for other agent find the station
            mode = Mode.WAIT;
        }
        else if (this.fuelStation == null) {
            mode = Mode.EXPLORE;
        }
        // if meet the refuel condition or already on the road of refueling
        else if ((this.fuelStation != null && this.getDistanceTo(fuelStation.x, fuelStation.y) >= this.fuelLevel * this.fuelThreshold) || this.mode==Mode.REFUEL) {
            // need to refuel
            mode = Mode.REFUEL;
        }
        else if(this.fuelStation != null && this.getDistanceTo(fuelStation.x, fuelStation.y) <= Parameters.defaultSensorRange*2 && this.fuelLevel<Parameters.defaultFuelLevel-Parameters.defaultSensorRange*15){
            mode = Mode.REFUEL;
        }
        else if (!this.hasTile()) {
            // if not tiles, collect the detected tiles if have
            if (tilesList.size() > 0) {
                mode = Mode.COLLECT;
            } else {
                mode = Mode.EXPLORE;
            }
        } else if (holesList.size() > 0) {
            //carried tiles upto limit || no tiles detected || tile farther than hole
            if (tilesList.size() == 0 ||
                    (this.getDistanceTo(tilesList.get(0)) > this.getDistanceTo(holesList.get(0)))
                    || this.carriedTiles.size() >= 3) {

                mode = Mode.FILL;
            } else {
                    mode = Mode.COLLECT;
            }
        }

        else if (this.carriedTiles.size() < 3 && tilesList.size() > 0) {
            //if there is only tile nearby
            mode = Mode.COLLECT;
        }

        // re-plan route and goal
        routePlanner.getGoalList().clear();
        routePlanner.voidPlan();

        /**
         * condition1: there is an object on the current position
         */
        Object cur_object = agentAreaMemory.getMemoryGrid().get(x, y);//get the object from memory(already updated)
        // if can pick up tile
        if (cur_object instanceof TWTile && this.carriedTiles.size() < 3
                && this.getEnvironment().canPickupTile((TWTile) cur_object, this)) {

            return new TWThought(TWAction.PICKUP, null);
        } else if (cur_object instanceof TWHole
                && this.carriedTiles.size() != 0
                && this.getEnvironment().canPutdownTile((TWHole) cur_object, this)) {

            return new TWThought(TWAction.PUTDOWN, null);
        } else if (cur_object instanceof TWFuelStation && this.fuelLevel < Parameters.defaultFuelLevel * 0.99) {
            this.mode = Mode.EXPLORE;//reset the mode
            return new TWThought(TWAction.REFUEL, null);
        }
        /**
         * condition2: there is no object on the current position
         * need to plan the route according to the Mode
         */
        else {
            if (mode == Mode.EXPLORE) {
                Int2D destPoint = null;
                //first: ensure the agent is in the range of the zone it belongs to
                if (zone_coordinates[0].x <= this.x && this.x <= zone_coordinates[1].x
                        && zone_coordinates[0].y <= this.y && this.y <= zone_coordinates[2].y) {
                    int goal_x = 0;
                    int goal_y = 0;
                    int cur_block_id = this.getCurBlockId();
                    if (cur_block_id == 0) {
                        //already back to the first block, set it to flag to false
                        this.isBackRoute = false;
                    }
                    // if reach the bottom right block or in the way of back_route, move back to top left block
                    if (cur_block_id == block_num - 1 || this.isBackRoute) {
                        goal_x = blocks[0].x;
                        goal_y = blocks[0].y;
                        this.isBackRoute = true;//set the backTrace
                        this.block_visited[cur_block_id] = false;
                    }
                    //move forward following the block order
                    else {
//                        int cur_block_id = this.getCurBlockId();
                        Int2D cur_block_center = blocks[cur_block_id];
                        if (this.x == cur_block_center.x && this.y == cur_block_center.y) {
                            this.block_visited[cur_block_id] = true;//set it true,mean the block center been visited
                        }
                        // if agent in the border block, need to move to it center point
                        //example: 0,6,7,13,14,20,21,27,28,34,35,41,42,48,49
                        if (cur_block_id % (this.hor_blocks) == 0 || cur_block_id % this.hor_blocks == (this.hor_blocks - 1) || cur_block_id == 0) {
                            //if the the center point of this border is blocked, set this border block has been visited(true)
                            if (this.agentAreaMemory.isCellBlocked(blocks[cur_block_id].x,blocks[cur_block_id].y)){
                                this.block_visited[cur_block_id] = true;
                            }
                            // move to center of the current block
                            if (this.block_visited[cur_block_id] == false) {
                                goal_x = blocks[cur_block_id].x;
                                goal_y = blocks[cur_block_id].y;
                            } else {
                                //already visited the center, then move to next block
                                goal_x = blocks[cur_block_id + 1].x;
                                goal_y = blocks[cur_block_id + 1].y;
                            }
                        } else {
                            this.block_visited[cur_block_id-1] = false;//set the previous block to no visited
                            goal_x = blocks[cur_block_id + 1].x;
                            goal_y = blocks[cur_block_id + 1].y;

                        }
                    }
                    //if the goal point is blocked, choose the surrounding one as alter
                    if (this.agentAreaMemory.isCellBlocked(goal_x,goal_y)){
                        Int2D alt_point = null;
                        for (int i=-1;i<=1;i++){
                            for (int j=-1;j<=1;j++){
                                if(goal_x+i<this.getEnvironment().getxDimension() && goal_y+j < this.getEnvironment().getyDimension() && goal_x+i>=0 && goal_y+j>=0
                                        && !this.agentAreaMemory.isCellBlocked(goal_x+i,goal_y+j)){
                                    //if no blocked
                                    alt_point = new Int2D(goal_x+i,goal_y+j);
                                    break;
                                }
                            }
                        }
                        destPoint = alt_point;
                    }
                    // goal point is not blocked
                    else {
                        destPoint = new Int2D(goal_x, goal_y);
                    }
                }
                // if not: go to the zone be assigned
                else {
//                    Int2D destPoint = new Int2D(this.x, this.y);
                    if (isDivideByHeight) {//move up or down
                        if (this.y > zone_coordinates[2].y) {//move up
                            int goal_x = this.x;
                            int goal_y = zone_coordinates[2].y - Parameters.defaultSensorRange;
                            //check whether it is blocked
                            if (this.agentAreaMemory.isCellBlocked(goal_x,goal_y)){
                                Int2D alt_point = null;
                                for (int i=-1;i<=1;i++){
                                    for (int j=-1;j<=1;j++){
                                        if(goal_x+i<this.getEnvironment().getxDimension() && goal_y+j < this.getEnvironment().getyDimension() && goal_x+i>=0 && goal_y+j>=0
                                                && !this.agentAreaMemory.isCellBlocked(goal_x+i,goal_y+j)){
                                            //if no blocked
                                            alt_point = new Int2D(goal_x+i,goal_y+j);
                                            break;
                                        }
                                    }
                                }
                                destPoint = alt_point;
                            }else{
                                destPoint = new Int2D(this.x, zone_coordinates[2].y - Parameters.defaultSensorRange);
                            }
                        } else if (this.y < zone_coordinates[0].y) {//move down
                            int goal_x = this.x;
                            int goal_y = zone_coordinates[0].y + Parameters.defaultSensorRange;
                            if (this.agentAreaMemory.isCellBlocked(goal_x,goal_y)){
                                Int2D alt_point = null;
                                for (int i=-1;i<=1;i++){
                                    for (int j=-1;j<=1;j++){
                                        if(goal_x+i<this.getEnvironment().getxDimension() && goal_y+j < this.getEnvironment().getyDimension() && goal_x+i>=0 && goal_y+j>=0
                                                && !this.agentAreaMemory.isCellBlocked(goal_x+i,goal_y+j)){
                                            //if no blocked
                                            alt_point = new Int2D(goal_x+i,goal_y+j);
                                            break;
                                        }
                                    }
                                }
                                destPoint = alt_point;
                            }else {
                                destPoint = new Int2D(this.x, zone_coordinates[0].y + Parameters.defaultSensorRange);
                            }
                        }
                    } else {//move left or right
                        if (this.x > zone_coordinates[2].x) {
                            int goal_x = zone_coordinates[2].x - Parameters.defaultSensorRange;
                            int goal_y = this.y;
                            if (this.agentAreaMemory.isCellBlocked(goal_x,goal_y)){
                                Int2D alt_point = null;
                                for (int i=-1;i<=1;i++){
                                    for (int j=-1;j<=1;j++){
                                        if(goal_x+i<this.getEnvironment().getxDimension() && goal_y+j < this.getEnvironment().getyDimension() && goal_x+i>=0 && goal_y+j>=0
                                                && !this.agentAreaMemory.isCellBlocked(goal_x+i,goal_y+j)){
                                            //if no blocked
                                            alt_point = new Int2D(goal_x+i,goal_y+j);
                                            break;
                                        }
                                    }
                                }
                                destPoint = alt_point;
                            }else {
                                destPoint = new Int2D(zone_coordinates[2].x - Parameters.defaultSensorRange, this.y);
                            }
                        } else if (this.x < zone_coordinates[0].x) {
                            int goal_x = zone_coordinates[0].x + Parameters.defaultSensorRange;
                            int goal_y = this.y;
                            if (this.agentAreaMemory.isCellBlocked(goal_x,goal_y)){
                                Int2D alt_point = null;
                                for (int i=-1;i<=1;i++){
                                    for (int j=-1;j<=1;j++){
                                        if(goal_x+i<this.getEnvironment().getxDimension() && goal_y+j < this.getEnvironment().getyDimension() && goal_x+i>=0 && goal_y+j>=0
                                                && !this.agentAreaMemory.isCellBlocked(goal_x+i,goal_y+j)){
                                            //if no blocked
                                            alt_point = new Int2D(goal_x+i,goal_y+j);
                                            break;
                                        }
                                    }
                                }
                                destPoint = alt_point;
                            }else {
                                destPoint = new Int2D(zone_coordinates[0].x + Parameters.defaultSensorRange, this.y);

                            }
                        }
                    }
                }
                routePlanner.getGoalList().add(destPoint);//add the destination to planner

            } else if (mode == Mode.REFUEL) {
                routePlanner.getGoalList().add(new Int2D(this.fuelStation.getX(), this.fuelStation.getY()));
            } else if (mode == Mode.COLLECT) {
                Int2D cur_pos = new Int2D(tilesList.get(0).getX(), tilesList.get(0).getY());
                routePlanner.getGoalList().add(cur_pos);
            } else if (mode == Mode.FILL) {
                Int2D cur_pos = new Int2D(holesList.get(0).getX(), holesList.get(0).getY());
                routePlanner.getGoalList().add(cur_pos);
            } else if (mode == Mode.WAIT) {
                return new TWThought(TWAction.MOVE, TWDirection.Z);//stay
            }
            routePlanner.generatePlan();
            if (!routePlanner.hasPlan()) {//if no plan, stay here
                return new TWThought(TWAction.MOVE, TWDirection.Z);
            }
            return new TWThought(TWAction.MOVE, routePlanner.execute());
        }
    }

    @Override
    protected void act(TWThought thought) {
        Int2D goal = routePlanner.getCurrentGoal();
        try {
            switch (thought.getAction()) {
                case MOVE:
                    move(thought.getDirection());
                    break;
                case PICKUP:
                    pickUpTile((TWTile) agentAreaMemory.getMemoryGrid().get(this.x, this.y));
                    routePlanner.getGoalList().clear();
                    // set the pick-up tile as null
//                    agentAreaMemory.removeAgentPercept(this.getX(),this.getY());
//                    agentAreaMemory.getMemoryGrid().set(this.getX(),this.getY(),null);
                    break;
                case REFUEL:
                    refuel();
                    routePlanner.getGoalList().clear();
                    break;
                case PUTDOWN:
                    putTileInHole((TWHole) agentAreaMemory.getMemoryGrid().get(this.x, this.y));
                    routePlanner.getGoalList().clear();
                    // set the hole as null in memory
//                    agentAreaMemory.removeAgentPercept(this.getX(),this.getY());
//                    agentAreaMemory.getMemoryGrid().set(this.getX(),this.getY(),null);
                    break;
            }
        } catch (CellBlockedException e) {
            e.printStackTrace();
            System.out.println("Cell is blocked. Current Position: " + Integer.toString(this.x) + ", " + Integer.toString(this.y));
        }
        System.out.println("Step " + this.getEnvironment().schedule.getSteps());
        System.out.println(name + " score: " + this.score);
        System.out.println("Assigned Zone: " + Integer.toString(zoneBelongToAgent[agentId]));
        System.out.println("Mode: " + mode.name());
        System.out.println("Position: " + Integer.toString(this.x) + ", " + Integer.toString(this.y));
        if (goal != null) {
            System.out.println("Goal: " + goal.x + ", " + goal.y);
        } else
            System.out.println("Goal: WAIT");
        System.out.println("Tiles: " + this.carriedTiles.size());
        System.out.println("Fuel Level: " + this.fuelLevel);
        System.out.println("Fuel Station:" + this.fuelStation);
        System.out.println("");
    }

    @Override
    public String getName() {
        return null;
    }
}
