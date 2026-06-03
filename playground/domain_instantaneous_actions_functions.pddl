(define (domain logistics-numeric)
  (:requirements :strips :typing :fluents :disjunctive-preconditions :negative-preconditions)
  
  (:types
    location vehicle package - object
    truck drone - vehicle
  )
  
  (:predicates
    (at-vehicle ?v - vehicle ?l - location)
    (at-package ?p - package ?l - location)
    (in-vehicle ?p - package ?v - vehicle)
    (connected ?l1 - location ?l2 - location)
    (vehicle-available ?v - vehicle)
    (package-fragile ?p - package)
    (location-warehouse ?l - location)
    (package-delivered ?p - package)
  )
  
  (:functions
    (distance ?l1 - location ?l2 - location)
    (fuel ?v - vehicle)
    (fuel-capacity ?v - vehicle)
    (fuel-rate ?v - vehicle)
    (package-weight ?p - package)
    (vehicle-load ?v - vehicle)
    (max-load ?v - vehicle)
    (total-distance)
    (total-fuel-consumed)
    (packages-count)
    (priority ?p - package)
  )
  
  ;; Acción: mover un vehículo (requiere combustible suficiente)
  (:action drive
    :parameters (?v - vehicle ?from - location ?to - location)
    :precondition (and
      (at-vehicle ?v ?from)
      (connected ?from ?to)
      (vehicle-available ?v)
      (>= (fuel ?v) (* (distance ?from ?to) (fuel-rate ?v)))
      (> (distance ?from ?to) 0)
    )
    :effect (and
      (not (at-vehicle ?v ?from))
      (at-vehicle ?v ?to)
      (decrease (fuel ?v) (* (distance ?from ?to) (fuel-rate ?v)))
      (increase (total-distance) (distance ?from ?to))
      (increase (total-fuel-consumed) (* (distance ?from ?to) (fuel-rate ?v)))
    )
  )
  
  ;; Acción: cargar paquete (respetando límite de peso)
  (:action load
    :parameters (?p - package ?v - vehicle ?l - location)
    :precondition (and
      (at-package ?p ?l)
      (at-vehicle ?v ?l)
      (vehicle-available ?v)
      (<= (+ (vehicle-load ?v) (package-weight ?p)) (max-load ?v))
      (> (package-weight ?p) 0)
    )
    :effect (and
      (not (at-package ?p ?l))
      (in-vehicle ?p ?v)
      (increase (vehicle-load ?v) (package-weight ?p))
    )
  )
  
  ;; Acción: descargar paquete
  (:action unload
    :parameters (?p - package ?v - vehicle ?l - location)
    :precondition (and
      (in-vehicle ?p ?v)
      (at-vehicle ?v ?l)
      (vehicle-available ?v)
      (> (vehicle-load ?v) 0)
    )
    :effect (and
      (not (in-vehicle ?p ?v))
      (at-package ?p ?l)
      (decrease (vehicle-load ?v) (package-weight ?p))
      (package-delivered ?p)
      (increase (packages-count) 1)
    )
  )
  
  ;; Acción: repostar (solo si no está lleno)
  (:action refuel
    :parameters (?v - vehicle ?l - location)
    :precondition (and
      (at-vehicle ?v ?l)
      (location-warehouse ?l)
      (vehicle-available ?v)
      (< (fuel ?v) (fuel-capacity ?v))
    )
    :effect (and
      (assign (fuel ?v) (fuel-capacity ?v))
    )
  )
  
  ;; Acción: cargar paquete de alta prioridad (solo si prioridad >= 5)
  (:action load-priority
    :parameters (?p - package ?v - vehicle ?l - location)
    :precondition (and
      (at-package ?p ?l)
      (at-vehicle ?v ?l)
      (vehicle-available ?v)
      (<= (+ (vehicle-load ?v) (package-weight ?p)) (max-load ?v))
      (>= (priority ?p) 5)
    )
    :effect (and
      (not (at-package ?p ?l))
      (in-vehicle ?p ?v)
      (increase (vehicle-load ?v) (package-weight ?p))
    )
  )
  
  ;; Acción: cargar paquete ligero (peso < 10)
  (:action load-light
    :parameters (?p - package ?v - vehicle ?l - location)
    :precondition (and
      (at-package ?p ?l)
      (at-vehicle ?v ?l)
      (vehicle-available ?v)
      (<= (+ (vehicle-load ?v) (package-weight ?p)) (max-load ?v))
      (< (package-weight ?p) 10)
    )
    :effect (and
      (not (at-package ?p ?l))
      (in-vehicle ?p ?v)
      (increase (vehicle-load ?v) (package-weight ?p))
    )
  )
  
  ;; Acción: transferir (solo si ambos vehículos tienen capacidad)
  (:action transfer
    :parameters (?p - package ?v1 - vehicle ?v2 - vehicle ?l - location)
    :precondition (and
      (in-vehicle ?p ?v1)
      (at-vehicle ?v1 ?l)
      (at-vehicle ?v2 ?l)
      (vehicle-available ?v1)
      (vehicle-available ?v2)
      (<= (+ (vehicle-load ?v2) (package-weight ?p)) (max-load ?v2))
      (> (vehicle-load ?v1) 0)
    )
    :effect (and
      (not (in-vehicle ?p ?v1))
      (in-vehicle ?p ?v2)
      (decrease (vehicle-load ?v1) (package-weight ?p))
      (increase (vehicle-load ?v2) (package-weight ?p))
    )
  )
  
  ;; Acción: viaje de emergencia (permite viajar con poco combustible si distancia <= 5)
  (:action emergency-drive
    :parameters (?v - vehicle ?from - location ?to - location)
    :precondition (and
      (at-vehicle ?v ?from)
      (connected ?from ?to)
      (vehicle-available ?v)
      (<= (distance ?from ?to) 5)
      (> (fuel ?v) 0)
    )
    :effect (and
      (not (at-vehicle ?v ?from))
      (at-vehicle ?v ?to)
      (assign (fuel ?v) 0)
      (increase (total-distance) (distance ?from ?to))
    )
  )
)