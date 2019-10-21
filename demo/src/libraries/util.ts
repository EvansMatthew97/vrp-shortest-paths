import distance from '@turf/distance';
import { convertLength } from '@turf/helpers';

export function getDistance(points: number[][], units = 'degrees') {
  let prev = points[0];
  let sum = 0;

  points.forEach(point => {
    if (point === prev) return;
    sum += distance(prev, point, units);
  });

  return sum;
}

/**
 * 
 * @param count Number of points to create
 * @param origin The [lng, lat] of the centre
 * @param distanceInKm Maximum distance (square shape) from the origin to create points
 */
export function createRandomPoints(count: number, origin: [number, number], distanceInKm: number) {
  const distanceInDegrees = convertLength(distanceInKm, 'kilometers', 'degrees');
  return Array.from({length: count}, (v, k) => {
    const lng = (Math.random() - 0.5) * distanceInDegrees * 2 + origin[0];
    const lat = (Math.random() - 0.5) * distanceInDegrees * 2 + origin[1];

    return [lng, lat];
  });
}