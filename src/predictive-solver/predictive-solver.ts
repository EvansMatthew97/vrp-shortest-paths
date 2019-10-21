import getDistance from '@turf/distance';
import lineSliceAlong from '@turf/line-slice-along';
import { lineString } from '@turf/helpers';
import { findTargetLineInterception } from './target-interception';

export interface TargetPath {
  speed: number,
  positions: Array<[number, number]>,
}

export class PredictiveSolver {
  /**
   * Finds all possible permutations of the given paths.
   * For each permutation, finds the intercept of the vehicle with the paths
   * and then chooses the most optimal route.
   * @param vehicleSpeedDeg Vehicle speed in degrees per second
   * @param maxRouteDistanceDeg vehicle maximum route distance in degrees
   * @param depotLon depot longitude
   * @param depotLat depot latitude
   * @param futurePaths future paths array. All speeds should be in degrees per second. Positions are [lon, lat]
   */
  solve(vehicleSpeedDeg: number, maxRouteDistanceDeg: number, depotLon: number, depotLat: number, futurePaths: Array<TargetPath>) {
    if (vehicleSpeedDeg <= 0) {
      return [];
    }

    // calculate all permutations of predictions (i.e. for each target)
    const permutations: Array<Array<TargetPath>> = [];
    const permute = (arr: any[], m: any[] = []) => {
      if (arr.length === 0) {
        permutations.push(m);
        return;
      }
      for (let i = 0; i < arr.length; i++) {
        const curr = arr.slice();
        const next: Array<Array<TargetPath>> = curr.splice(i, 1);
        permute(curr.slice(), m.concat(next));
      }
    };
    permute(futurePaths);


    // now go through permutations and find the best route to take

    let bestNumPointsReached = -Infinity;
    let bestDistance = Infinity;
    let bestPath = null; // the best path found so far

    // now try find paths from depot to points, taking into account maximum distance
    for (const permutation of permutations) {
      const paths = []; // store the paths we find - will have more than one path if the maximum distance is exceeded
      let totalDistance = 0; // total distance of all paths. Used to sort the results
      let distanceElapsed = 0; // distance of the current path

      let activePath: Array<[number, number]> = [[depotLon, depotLat]]; // path to build up - starts at depot
      let lastPosition = activePath[0];

      // iterate over each path in the permutation (path of [lon, lat])
      for (const predictedPath of permutation) {
        // find the interception point to the path from the last vehicle intercept
        const interceptPoint = this.findLineIntercept(
          lastPosition,
          distanceElapsed / vehicleSpeedDeg,
          vehicleSpeedDeg,
          predictedPath,
        );

        // if the point cannot be reached, move on to the next point
        if (typeof interceptPoint === 'undefined') {
          continue;
        }

        // distance from the last point to the predicted point
        // if the distance to the point from the last point and then back to the depot
        // is too great, then return to the depot immediately and start a new route
        const pointDistance = getDistance(lastPosition, interceptPoint, { units: 'degrees' });
        const distanceBackToDepot = getDistance(interceptPoint, [depotLon, depotLat], { units: 'degrees' });

        // if the distance + the distance back to depot exceeds max distance, return to depot and exclude point
        if (distanceElapsed + pointDistance + distanceBackToDepot > maxRouteDistanceDeg) {
          // end this path
          activePath.push([depotLon, depotLat]);
          paths.push(activePath);
          // add the distance from the last feasible point back to the depot
          totalDistance += getDistance(lastPosition, [depotLon, depotLat], { units: 'degrees' });

          // create a new path
          // first we need to find the intercept from the depot to the new point at elapsed = 0
          const newInterceptPoint = this.findLineIntercept(
            [depotLon, depotLat],
            0,
            vehicleSpeedDeg,
            predictedPath,
          );

          if (typeof newInterceptPoint === 'undefined') {
            continue;
          }

          // create the active path
          activePath = [[depotLon, depotLat], newInterceptPoint];
          // add the distance from the depot to the new intercept
          const distanceDepotToNewIntercept = getDistance([depotLon, depotLat], newInterceptPoint, { units: 'degrees' });
          lastPosition = newInterceptPoint;
          distanceElapsed = distanceDepotToNewIntercept;
          totalDistance += distanceDepotToNewIntercept;
        }
        // otherwise, simply add the intercept point
        else {
          activePath.push(interceptPoint);
          lastPosition = interceptPoint;
          distanceElapsed += pointDistance;
          totalDistance += pointDistance;
        }
      }

      // if the last path did not exceed the maximum distance, add it
      if (paths.indexOf(activePath) === -1 && activePath.length > 1) {
        activePath.push([depotLon, depotLat]);
        paths.push(activePath);
      }

      const feasiblePaths = this.filterFeasibleRoutes(paths, maxRouteDistanceDeg);

      const numPointsReached = feasiblePaths.reduce((total, route) => {
        return total + route.length - 2; // - 2 to exclude depots
      }, 0);

      // determine whether to set as the best path - if it visits more points, then it is better
      if (
        bestPath === null || // no best path set yet
        numPointsReached > bestNumPointsReached || // reaches more points so it has to be better
        (numPointsReached === bestNumPointsReached && totalDistance < bestDistance) // reaches same number of points but lower distance
      ) {
        bestPath = feasiblePaths;
        bestDistance = totalDistance;
        bestNumPointsReached = numPointsReached;
      }
    }

    return bestPath;
  }

  /**
   * Filers feasibly routes by ensuring that they aren't only between the depot back to itself
   * and that they are within the maximum route distance of the vehicle.
   * @param paths 
   * @param maxRouteDistanceDeg 
   */
  private filterFeasibleRoutes(paths: Array<Array<[number, number]>>, maxRouteDistanceDeg: number) {
    return paths.filter(route => {
      if (route.length <= 2) {
        // if it's only depot to depot, don't count it
        return false;
      }

      const routeDistance = route.reduce((sum, point, index) => {
        if (index === route.length - 1) {
          return sum;
        }
        const pointDistance = getDistance(point, route[index + 1], { units: 'degrees' });
        return sum + pointDistance;
      }, 0);

      return routeDistance < maxRouteDistanceDeg;
    });
  }

  /**
   * Find the intercept between a line and the vehicle given a time offset.
   * To calculate the intercept, we treat the path the target goes on
   * as multiple lines. Each of these lines is tested for an intersection
   * point. If there is no intersection point, it tests the next line
   * segment in the path.
   *
   * We find the intercept as follows:
   * We know where the target line starting point is and what its direction
   * is and its velocity.
   * We also know where the vehicle starts and what its velocity is. We treat
   * the area the vehicle can reach as a circle.
   * We then find the intercept between the circle and the target's path.
   * @param vehiclePosition 
   * @param timeElapsed 
   * @param vehicleSpeedDeg 
   * @param targetSpeedDeg 
   * @param targetPositions 
   */
  private findLineIntercept(vehiclePosition: [number, number], timeElapsed: number, vehicleSpeedDeg: number, targetPath: TargetPath): [number, number] | undefined {
    // remove parts of the path the target has already moved along
    const lines = this.trimTargetPath(timeElapsed, targetPath);

    // account for any error with trimming the line
    if (typeof lines === 'undefined') {
      return undefined;
    }

    let elapsedDistance = 0;
    for (const line of lines) {
      const interception = findTargetLineInterception(
        line,
        targetPath.speed,
        vehiclePosition,
        vehicleSpeedDeg,
        elapsedDistance / targetPath.speed, // time delay of how long the target has moved
      );

      // check that the intercept point is actually on the line
      const lineDistance = getDistance(line[0], line[1], { units: 'degrees' });
      const interceptionDistance = getDistance(line[0], [interception.getX(), interception.getY()], { units: 'degrees' });

      elapsedDistance += lineDistance;

      // if the point isn't on the line, then try the next line segment
      if (interceptionDistance > lineDistance) {
        continue;
      }

      // if the point is on the line, then return the interception point
      return [interception.getX(), interception.getY()];
    }

    // no point was found so return undefined
    return undefined;
  }

  /**
    * Trims a target path by removing the parts it has already travelled along
    * @param timeElapsed How much time (in seconds) the target has moved for
    * @param targetPath The target path
    */
  private trimTargetPath(timeElapsed: number, targetPath: TargetPath) {
    // how far the target has travelled
    const targetDistanceTravelled = timeElapsed * targetPath.speed;
  
    // trim the path to remove parts of the path the target has already moved along
    try {
      var offsetLine = lineSliceAlong(
        lineString(targetPath.positions),
        targetDistanceTravelled,
        Infinity,
        { units: 'degrees' },
      );
    } catch (err) {
      console.error('line error', err);
      return undefined;
    }
  
    // if the resulting lineString is empty, no intercept can be found
    if (!offsetLine.geometry || !offsetLine.geometry.coordinates) {
      return undefined;
    }
  
    const points: Array<[number, number]> = offsetLine.geometry.coordinates as Array<[number, number]>;
  
    // convert points into a line of point pairs
    return points.reduce((lineAcc: Array<[[number, number], [number, number]]>, point, pointIndex) => {
      if (pointIndex === points.length - 1) {
        return lineAcc;
      }
      lineAcc.push([point, points[pointIndex + 1]]);
      return lineAcc;
    }, []);
  }
}