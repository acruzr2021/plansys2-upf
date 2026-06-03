(define (domain robot-logistics)
  (:requirements :strips :typing :fluents :durative-actions)
  (:types robot location package)
  (:predicates
    (robot_at ?r - robot ?l - location)
    (package_at ?p - package ?l - location)
    (carrying ?r - robot ?p - package)
    (connected ?l1 ?l2 - location)
    (charging_point_at ?l - location)
  )
  (:functions
    (battery ?r - robot)
    (move_cost ?l1 ?l2 - location)
    (packages_carried ?r - robot)
    (total_deliveries)
  )
  (:durative-action move
    :parameters (?r - robot ?from ?to - location)
    :duration (= ?duration 5)
    :condition (and
      (at start (robot_at ?r ?from))
      (at start (connected ?from ?to))
    )
    :effect (and
      (at start (not (robot_at ?r ?from)))
      (at end (robot_at ?r ?to))
      (at end (decrease (battery ?r) (move_cost ?from ?to)))
    )
  )
  (:durative-action pickup
    :parameters (?r - robot ?p - package ?l - location)
    :duration (= ?duration 2)
    :condition (and
      (at start (robot_at ?r ?l))
      (at start (package_at ?p ?l))
    )
    :effect (and
      (at end (not (package_at ?p ?l)))
      (at end (carrying ?r ?p))
      (at end (increase (packages_carried ?r) 1))
    )
  )
  (:durative-action deliver
    :parameters (?r - robot ?p - package ?l - location)
    :duration (= ?duration 2)
    :condition (and
      (at start (robot_at ?r ?l))
      (at start (carrying ?r ?p))
    )
    :effect (and
      (at end (not (carrying ?r ?p)))
      (at end (package_at ?p ?l))
      (at end (decrease (packages_carried ?r) 1))
      (at end (increase (total_deliveries) 1))
    )
  )
  (:durative-action charge
    :parameters (?r - robot ?l - location)
    :duration (= ?duration 5)
    :condition (and
      (at start (robot_at ?r ?l))
      (at start (charging_point_at ?l))
    )
    :effect (at end (increase (battery ?r) 100))
  )
)