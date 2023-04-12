const int kCost1 = 10;
const int kCost2 = 14;   

struct Point{
    double x, y;
	int F, G, H;
	Point *parent;
	Point(double _x, double _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL){}
	int stageId =-1;
	int zzposition = -1;
	int begin_stageId =-1;
	int end_stageId = -1;
	Point(){};
};
struct compare1{
	bool operator()(Point *a, Point *b){
		return a->F > b->F;
	}
};

class Astar{
    public:
        void Init(vector<vector<char>> &_map);
	    list<Point *> GetPath_1(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
	    list<Point *> GetPath_1_5(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
    private:
        Point *findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
        Point *findPath1(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
        vector<Point *> getSurroundPoints(const Point *point, bool isIgnoreCorner) const;
        vector<Point *> getSurroundPoints1(const Point *point, bool isIgnoreCorner) const;
        bool isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断
        bool isCanreach1(const Point *point, const Point *target, bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断
		pair<double, double> getKey(double x, double y) const;
		void addKey(set<pair<double, double>> *set, double x, double y) const;
		void removeKey(set<pair<double, double>> *set, double x, double y) const;
		bool contains(const set<pair<double, double>> &set, double x, double y) const;
		Point *findOpenList(const std::priority_queue<Point *, vector<Point *>, compare1> &list, double x, double y);
        //计算FGH值
        int calcG(Point *temp_start, Point *point);
        int calcH(Point *point, Point *end);
        int calcF(Point *point);
    private:
        vector<vector<char>> map;
        priority_queue<Point *, vector<Point *>, compare1> openList;  //开启列表
		set<pair<double, double>> openSet;
		set<pair<double, double>> closeSet;
};

void Astar::Init(vector<vector<char>> &_map){
	map = _map;
}
 
int Astar::calcG(Point *temp_start, Point *point){
	double extraG = (fabs(point->x - temp_start->x) + fabs(point->y - temp_start->y)) == 0.5 ? kCost1 : kCost2;
	return extraG;
}
 
int Astar::calcH(Point *point, Point *end){
	double a = sqrt((double)(end->x - point->x)*(double)(end->x - point->x) + (double)(end->y - point->y)*(double)(end->y - point->y)) * kCost1;
	return a / 2;
}
 
int Astar::calcF(Point *point){
	return point->G + point->H;
}
 
pair<double, double> Astar::getKey(double x, double y) const{
	return {x, y};
}

void Astar::addKey(set<pair<double, double>> *set, double x, double y) const{
	set->insert(getKey(x, y));
}

void Astar::removeKey(set<pair<double, double>> *set, double x, double y) const{
	set->erase(getKey(x, y));
}

bool Astar::contains(const set<pair<double, double>> &set, double x, double y) const{
	if(set.find(getKey(x, y)) != set.end())
		return true;
	else
		return false;
}

Point *Astar::findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner){
	openList.push(new Point(startPoint.x + 0.25, startPoint.y + 0.25)); //置入起点,拷贝开辟一个节点，内外隔离
	while (!openList.empty()){
		auto minPoint = openList.top();
		openList.pop();
		removeKey(&openSet, minPoint->x, minPoint->y);
		addKey(&closeSet, minPoint->x, minPoint->y);
		auto surroundPoints = getSurroundPoints(minPoint, isIgnoreCorner);
		for (auto &target : surroundPoints){
			//2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
			int G = calcG(minPoint, target) + minPoint->G;
			if (!contains(openSet, target->x, target->y)){
				target->parent = minPoint;
				target->G = G;
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);
				addKey(&openSet, target->x, target->y);
				openList.push(target);
			}
			//3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
			else{
				if (G < target->G){
					target->parent = minPoint;
					target->G = G;
					target->F = calcF(target);
				}
			}
			for(int i = -1; i <= 1; i += 2){
				for(int j = -1; j <= 1; j += 2){
					if(contains(openSet, endPoint.x + i * 0.25, endPoint.y + j * 0.25)){
						auto a = findOpenList(openList, endPoint.x + i * 0.25, endPoint.y + j * 0.25);
						auto point = new Point(endPoint.x, endPoint.y);
						point->parent = a;
						point->G = calcG(a, point) + a->G;
						point->H = calcH(a, point);
						point->F = calcF(a);
						addKey(&openSet, point->x, point->y);
						openList.push(point);
						return findOpenList(openList, point->x, point->y);
					}
				}
			}
		}
	}
 
	return NULL;
}
 
std::list<Point *> Astar::GetPath_1(Point &startPoint, Point &endPoint, bool isIgnoreCorner){
	Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
	std::list<Point *> path;
	//返回路径，如果没找到路径，返回空链表
	while (result){
		path.push_front(result);
		result = result->parent;
	}
	// 清空临时开闭列表，防止重复执行GetPath_1导致结果异常
	openList = priority_queue <Point *, vector<Point *>, compare1>();
	openSet.clear();
	closeSet.clear();
 
	return path;
}
 
Point *Astar::findOpenList(const std::priority_queue<Point *, vector<Point *>, compare1> &list, double x, double y){
	priority_queue<Point *, vector<Point *>, compare1> list1 = list;
	for(int i = 0; i < list.size(); i++){
		if(list1.top()->x == x && list1.top()->y == y)
			return list1.top();
		list1.pop();
	}
	return NULL;
} 
 
bool Astar::isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const{
	if (target->x <= 0 || target->x >= 50 || target->y <= 0 || target->y >= 50
		|| map[(49.5 - target->y) * 2][target->x * 2] == '#' || map[(49.5 - target->y) * 2 + 1][target->x * 2 - 1] == '#' 
		|| map[(49.5 - target->y) * 2][target->x * 2 - 1] == '#' || map[(49.5 - target->y) * 2 + 1][target->x * 2] == '#' 
		|| target->x == point->x && target->y == point->y 
        || contains(closeSet, target->x, target->y)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
		return false;
	else{
		if (abs(point->x - target->x) + abs(point->y - target->y) == 0.5) //非斜角可以
			return true;
		else{
			//斜对角要判断是否绊住
			if (map[(49.5 - target->y) * 2 + 1][point->x * 2 - 1] == '.' ||
                map[(49.5 - point->y) * 2][target->x * 2] == '.')
				return true;
			else
				return isIgnoreCorner;
		}
	}
    return false;
}
 
std::vector<Point *> Astar::getSurroundPoints(const Point *point, bool isIgnoreCorner) const{
	std::vector<Point *> surroundPoints;
	for(double x = point->x - 0.5; x <= point->x + 0.5; x += 0.5)
	for(double y = point->y - 0.5; y <= point->y + 0.5; y += 0.5)
	if(isCanreach(point, new Point(x, y), isIgnoreCorner))
		surroundPoints.push_back(new Point(x, y));
	return surroundPoints;
}

std::list<Point *> Astar::GetPath_1_5(Point &startPoint, Point &endPoint, bool isIgnoreCorner){
	Point *result = findPath1(startPoint, endPoint, isIgnoreCorner);
	std::list<Point *> path;
	//返回路径，如果没找到路径，返回空链表
	while (result){
		path.push_front(result);
		result = result->parent;
	}
	// 清空临时开闭列表，防止重复执行GetPath_1导致结果异常
	openList = priority_queue <Point *, vector<Point *>, compare1>();
	openSet.clear();
	closeSet.clear();
 
	return path;
}

Point *Astar::findPath1(Point &startPoint, Point &endPoint, bool isIgnoreCorner){
	openList.push(new Point(startPoint.x, startPoint.y)); //置入起点,拷贝开辟一个节点，内外隔离
	while (!openList.empty()){
		auto minPoint = openList.top();
		openList.pop();
		removeKey(&openSet, minPoint->x, minPoint->y);
		addKey(&closeSet, minPoint->x, minPoint->y);
		auto surroundPoints = getSurroundPoints1(minPoint, isIgnoreCorner);
		for (auto &target : surroundPoints){
			//2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H
			double G = calcG(minPoint, target) + minPoint->G;
			if (!contains(openSet, target->x, target->y)){
				target->parent = minPoint;
				target->G = G;
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);
				addKey(&openSet, target->x, target->y);
				openList.push(target);
			}
			//3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
			else{
				if (G < target->G){
					target->parent = minPoint;
					target->G = G;
					target->F = calcF(target);
				}
			}
			if (contains(openSet, endPoint.x, endPoint.y))
				return findOpenList(openList, endPoint.x, endPoint.y);
		}
	}
 
	return NULL;
}

bool Astar::isCanreach1(const Point *point, const Point *target, bool isIgnoreCorner) const{
	if (target->x <= 0.25 || target->x >= 49.75 || target->y <= 0.25 || target->y >= 49.75
		|| (map[(49.75 - target->y) * 2 - 1][(target->x - 0.25) * 2 - 1] == '#' && map[(49.75 - target->y) * 2 + 1][(target->x - 0.25) * 2 + 1] == '#')
        || (map[(49.75 - target->y) * 2 - 1][(target->x - 0.25) * 2 + 1] == '#' && map[(49.75 - target->y) * 2 + 1][(target->x - 0.25) * 2 - 1] == '#')
		|| map[(49.75 - target->y) * 2 - 1][(target->x - 0.25) * 2] == '#' || map[(49.75 - target->y) * 2 + 1][(target->x - 0.25) * 2] == '#'
		|| map[(49.75 - target->y) * 2][(target->x - 0.25) * 2 - 1] == '#' || map[(49.75 - target->y) * 2][(target->x - 0.25) * 2 + 1] == '#'
		|| map[(49.75 - target->y) * 2][(target->x - 0.25) * 2] == '#' || target->x == point->x && target->y == point->y 
        || contains(closeSet, target->x, target->y)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
		return false;
	else{
		if (abs(point->x - target->x) + abs(point->y - target->y) == 0.5) //非斜角可以
			return true;
		else{
			//斜对角要判断是否绊住
			if((map[(49.75 - target->y) * 2 + 1][(target->x - 0.25) * 2 - 1] == '#' && map[(49.75 - point->y) * 2 - 1][(point->x - 0.25) * 2 + 1] == '#') ||
			    map[(49.75 - point->y) * 2 + 1][(point->x - 0.25) * 2 - 1] == '#' && map[(49.75 - target->y) * 2 - 1][(target->x - 0.25) * 2 + 1] == '#')
				return false;
			else if (map[(49.75 - target->y) * 2 + 1][(point->x - 0.25) * 2 - 1] == '.' ||
                map[(49.75 - point->y) * 2 - 1][(target->x - 0.25) * 2 + 1] == '.')
				return true;
			else
				return isIgnoreCorner;
		}
	}
    return false;
}

std::vector<Point *> Astar::getSurroundPoints1(const Point *point, bool isIgnoreCorner) const{
	std::vector<Point *> surroundPoints;
	for(double x = point->x - 0.5; x <= point->x + 0.5; x += 0.5)
	for(double y = point->y - 0.5; y <= point->y + 0.5; y += 0.5)
	if(isCanreach1(point, new Point(x, y), isIgnoreCorner))
		surroundPoints.push_back(new Point(x, y));
	return surroundPoints;
}