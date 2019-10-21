import getDistance from '@turf/distance';

/**
 * Options for a Clarke Wright problem
 */
export interface ClarkeWrightProblemOptions {
  /** List of points to visit */
  customers: Customer[];
  
  /** Depot location (lon, lat) */
  depot: { lon: number, lat: number };

  /** Maximum distance (in degrees) that the vehicle can travel (d = vt) */
  maxDistance: number;
}

/**
 * Interface for points
 */
export interface Customer {
  lon: number;
  lat: number;
  demand: number;
}

/**
 * Weights used for heuristics in the Clarke Wright algorithm
 */
export interface CWWeights {
  /** How important adjacent vertices are (0.1 to 2) */
  adjacency: number;

  /** How important points being on the same side of the depot is (0 to 2)  */
  asymmetry: number;

  /** Importance of demand (0 to 2) */
  demand: number;

  /** Minimum savings (0 to 0.5) - higher can yield more optimal routes  */
  minSavings: number;
}

/**
 * Clarke Wright Vehicle Routing Problem solver
 * Uses an altered version of the classic CW heuristric with
 * additional heuristics:
 * - adjacency - how close points are to each other
 * - symmetry - measure of whether points are on the same side of the depot
 * - demand - how above demand the points are vs the rest of the points
 */
export class ClarkeWrightProblem {
  private points: Point[] = [];
  private maxDistance: number;
  private depot: Point;
  private solutions: Route[] = []; // list of found solution routes
  private savings: Savings[] = []; // stores route savings pairs
  private joinedRoutes: { [id: number]: boolean } = {}; // map of joined routes
  private averageDemand: number; // average demand of all points
  private maxDemand: number; // maximum demand of all points
  private maxPointDistance: number; // maximum distance between any two points

  /**
   * Constructs a Clarke Wright prolem with the given data points
   * @param points List of points to visit
   * @param depot Depot location
   * @param maxDistance Maximum distance the vehicle can travel. Must be in the same unit as x, y
   */
  constructor(options: ClarkeWrightProblemOptions) {
    // set problem options
    this.depot = new Point(options.depot.lon, options.depot.lat, 0);
    this.maxDistance = options.maxDistance;

    this.points = options.customers.map(customer => Point.fromCustomer(customer));
    this.points = this.points.sort(() => Math.random() - 0.5); // randomise point order TODO: make automated in solve

    // calculate static properties use throughout calculations
    // (max point distance, average demand, maximum demand)
    this.maxPointDistance = -Infinity;
    for (const point of this.points) {
      for (const otherPoint of this.points) {
        const distance = getDistance([point.x, point.y], [otherPoint.x, otherPoint.y], { units: 'degrees' });
        point.setDistanceTo(otherPoint, distance);
        if (distance > this.maxPointDistance) {
          this.maxPointDistance = distance;
        }
      }

      const depotDistance = getDistance([point.x, point.y], [this.depot.x, this.depot.y], { units: 'degrees' });
      point.setDistanceTo(this.depot, depotDistance);
      this.depot.setDistanceTo(point, depotDistance);
    }

    this.averageDemand = this.points.reduce((sum, point) => sum + point.demand, 0) / this.points.length;
    this.maxDemand = this.points.reduce((max, point) => max < point.demand ? point.demand : max, -Infinity);
  }

  /**
   * Solves the problem, returning a list of possible routes sorted in order
   * of best demand vs distance
   * @param weights Clarke Wright weight
   * @param optimise
   */
  public solve(weights: CWWeights, optimise = true) {
    // first, create one route per point
    this.solutions = this.points.map(point => new Route(point, this.depot));
    // then calculate savings
    this.findAllRouteSavingsPairs(weights);

    // now go over savings and choose best pairs of routes to join
    while (this.savings.length) {
      for (const savings of this.savings) {
        const { routeA, routeB } = savings;

        // if neither route has been joined, join them
        if (!this.joinedRoutes[routeA.id] && !this.joinedRoutes[routeB.id]) {
          this.joinRoutes(routeA, routeB);
        } else if (!this.joinedRoutes[routeA.id]) {
          if (this.solutions.length && this.solutions[0].getStart() === routeB.getStart()) {
            this.joinRoutes(routeA, this.solutions[0]);
          }
        } else if (!this.joinedRoutes[routeB.id]) {
          if (this.solutions.length && this.solutions[0].getEnd() === routeA.getEnd()) {
            this.joinRoutes(this.solutions[0], routeB);
          }
        }
      }

      // routes may contain overlapping edges. Iron them out if option is enabled
      if (optimise) {
        this.solutions.forEach(solution => solution.optimise());
      }

      this.joinedRoutes = {}; // clear joined routes map

      // recalculate savings
      this.findAllRouteSavingsPairs(weights);
    }

    // sort the routes by their demand served
    return this.solutions
      .filter(route => route.totalDistance() < this.maxDistance) // some routes might be too long for outlier points
      .sort((a, b) => b.totalDemand() / b.totalDistance() - a.totalDemand() / a.totalDistance());
  }

  /**
   * Calculates savings for all routes and sorts them in descending order
   * (higher savings is better)
   */
  private findAllRouteSavingsPairs(weights: CWWeights) {
    this.savings = this.solutions.reduce((arr: Savings[], route) => {
      this.solutions.forEach(otherRoute => {
        // if they're the same route they can't be a pair
        if (otherRoute === route) {
          return;
        }

        // find the savings between the start of one route and the end of the other
        const savingsAB = this.calculateSavings(route, otherRoute, weights);
        const savingsBA = this.calculateSavings(otherRoute, route, weights);

        // we want to use the greatest savings
        const a = savingsAB > savingsBA ? route : otherRoute;
        const b = (a === route ? otherRoute : route);
        const savings = Math.max(savingsAB, savingsBA);

        // discard savings if under minimum savings weight
        if (savings < weights.minSavings) {
          return;
        }

        // if the route is feasible (doesn't exceed the maximum distance), add it to the savings list
        if (this.verifyRouteJoin(a, b)) {
          arr.push({
            routeA: a,
            routeB: b,
            savings,
          });
        }
      });
      return arr;
    }, [])
    .sort((a, b) => b.savings - a.savings); // sort the savings in descending order
  }

  /**
   * Calculate the savings between two routes.
   * Uses the end of routeA and the start of routeB
   * @param routeA
   * @param routeB
   */
  private calculateSavings(routeA: Route, routeB: Route, weights: CWWeights): number {
    const customerA = routeA.getEnd();
    const customerB = routeB.getStart();

    const distDA = this.depot.getDistanceTo(customerA); // depot to A
    const distDB = this.depot.getDistanceTo(customerB); // depot to B

    const distAB = customerA.getDistanceTo(customerB); // A to B

    // adjacency measure = cost of going depot -> A -> B -> depot
    // vs depot -> A -> depot -> B -> depot
    const savingsAdjacency = (distDA + distDB - weights.adjacency * distAB) / this.maxPointDistance;

    // asymmetry measure = distance between two customers vs their angle through the depot
    const savingsAsymmetry = weights.asymmetry * (
        this.angle(customerA, this.depot, customerB) *
        Math.abs(this.maxPointDistance - (distDA - distDB) / 2)
      ) / this.maxPointDistance;

    // the average demand of the two points vs the average demand of all points
    const savingsDemand = weights.demand * Math.abs((this.averageDemand - (customerA.demand + customerB.demand) / 2)) / this.maxDemand;

    return savingsAdjacency + savingsAsymmetry + savingsDemand;
  }

  /**
   * Join two routes together
   * @param routeA
   * @param routeB
   */
  private joinRoutes(routeA: Route, routeB: Route): boolean {
    if (!this.verifyRouteJoin(routeA, routeB)) {
      // if the routes can't be joined because they would exceed the maximum
      // distance, return
      return false;
    }

    // join the routes
    routeA.joinRoute(routeB);

    // indicate that the routes have been joined
    this.joinedRoutes[routeA.id] = true;
    this.joinedRoutes[routeB.id] = true;

    // remove route B from the routes
    this.solutions.splice(this.solutions.indexOf(routeB), 1);
    return true;
  }

  /**
   * Determines whether the total length of the route would
   * exceed the maximum distance that can be travelled
   * @param routeA
   * @param routeB
   */
  private verifyRouteJoin(routeA: Route, routeB: Route) {
    const distance = routeA.totalDistance() - // total of A
      routeA.getEnd().getDistanceTo(this.depot) + // minus end of A to depot
      routeA.getEnd().getDistanceTo(routeB.getStart()) + // end of A and start of B
      routeB.totalDistance() - // total of B
      routeB.getStart().getDistanceTo(this.depot); // minus depot to start of B

    return distance < this.maxDistance;
  }

  /**
   * Calculates cos of the angle between p0 and p2 through p1
   * @param p0
   * @param p1
   * @param p2
   */
  private angle(p0: Point, p1: Point, p2: Point): number {
    const a = Math.pow(p1.x - p0.x, 2) + Math.pow(p1.y - p0.y, 2);
    const b = Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2);
    const c = Math.pow(p2.x - p0.x, 2) + Math.pow(p2.y - p0.y, 2);

    return Math.cos(Math.acos((a + b - c) / Math.sqrt(4 * a * b)));
  }
}

/**
 * A Clarke Wright algorithm point
 */
export class Point {
  x: number;
  y: number;
  demand: number;
  private distances = new WeakMap<Point, number>();

  /**
   * Instantiates a point
   * @param x The x position of the point
   * @param y The y position of the point
   * @param demand The demand/priority of the point - higher means more demand
   */
  constructor(x: number, y: number, demand: number) {
    this.x = x;
    this.y = y;
    this.demand = demand;
  }

  /**
   * Converts a customer interface into a point
   * @param json 
   */
  public static fromCustomer(customer: Customer) {
    return new Point(customer.lon, customer.lat, customer.demand);
  }

  /**
   * Calculates the euclidean distance between two points
   * @param point
   */
  getDistanceTo(point: Point): number {
    const distance = this.distances.get(point);
    if (typeof distance === 'undefined') {
      return Infinity;
    }

    return distance;
  }

  setDistanceTo(point: Point, distance: number) {
    this.distances.set(point, distance);
  }
}

/**
 * Clarke Wright route
 */
class Route {
  static id = 0;
  points: Point[]; // list of points excluding the depot
  depot: Point; // the depot

  id: number;

  constructor(a: Point, depot: Point) {
    this.points = [depot, a];
    this.depot = depot;
    this.id = Route.id++;
  }

  /**
   * Returns the start point of the route (after the depot)
   */
  public getStart(): Point {
    return this.points[1];
  }

  /**
   * Returns the end point of the route (before the depot)
   */
  public getEnd(): Point {
    return this.points[this.points.length - 1];
  }

  /**
   * Appends the points of the given route to this route (excluding
   * the depot)
   * @param route 
   */
  public joinRoute(route: Route) {
    this.points.push(...route.points.slice(1));
  }

  /**
   * Cost from the depot through the points and back to the depot
   */
  public totalDistance(): number {
    let prev = this.points[0];
    let cost = 0;

    this.points.forEach(point => {
      if (point === prev) return;
      cost += prev.getDistanceTo(point);
      prev = point;
    });

    cost += prev.getDistanceTo(this.depot);

    return cost;
  }

  /**
   * Total demand of all the points
   */
  public totalDemand() {
    return this.points.reduce((sum, point) => sum + (point === this.depot ? 0 : point.demand), 0);
  }

  /**
   * Attempts to optimise the route using local search.
   * Runs until no further improvements are made.
   * @param iterations The number of times to re-run
   */
  public optimise(maxIter = 100) {
    let bestDistance = 0;
    for (let i = 0; i < maxIter; i++) {
      const distance = this.twoOpt();
      if (distance === bestDistance) {
        break;
      }
      bestDistance = distance;
    }
  }

  /**
   * Two-opt implementation. Swaps points if the distances
   * would be improved. Usually gets rid of most intersecting edges.
   * This is exhaustive.
   * 
   * This implementation takes a circuit and reverses the part between the
   * given edges rather than actually swapping the edges. This has the same
   * effect.
   * 
   * A ---   --- B            A --------> B
   * ^    \ /    ^            ^           |
   * |    / \    |  becomes   |           v
   * D <--   --> C            D <-------- C
   * 
   * A->C->B->D->A            A->B->C->D->A
   * 
   * Returns the total distance so we can compare with best distance.
   */
  private twoOpt() {
    for (let i = 0; i < this.points.length - 2; i++) {
      for (var j = i + 2; j < this.points.length - 1; j++) {
        if (j - i === 1) continue;
        if (
          this.points[i].getDistanceTo(this.points[i + 1]) + this.points[j].getDistanceTo(this.points[j + 1])
          > this.points[i].getDistanceTo(this.points[j]) + this.points[j + 1].getDistanceTo(this.points[i + 1])
        ) {
          this.twoOptSwap(i, j);
          return this.totalDistance();
        }
      }

      // test last point back to first point
      if (
        this.points[i].getDistanceTo(this.points[i + 1]) + this.points[j].getDistanceTo(this.points[0])
        > this.points[i].getDistanceTo(this.points[j]) + this.points[0].getDistanceTo(this.points[i + 1])
      ) {
        this.twoOptSwap(i, j);
        return this.totalDistance();
      }
    }
    return this.totalDistance();
  }

  /**
   * Swap function for two-opt
   */
  private twoOptSwap(i: number, k: number) {
    this.points = [
      ...this.points.slice(0, i + 1),
      ...this.points.slice(i + 1, k + 1).reverse(),
      ...this.points.slice(k + 1),
    ];
  }
}

/**
 * Represents savings between two routes
 */
interface Savings {
  savings: number;
  routeA: Route;
  routeB: Route;
}

