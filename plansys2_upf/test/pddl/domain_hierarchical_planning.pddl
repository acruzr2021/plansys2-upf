(define (domain robot-delivery-htn)
  (:requirements :strips :typing :hierarchy :method-preconditions)
  (:types robot location package)
  (:predicates
    (robot_at ?r - robot ?l - location)
    (package_at ?p - package ?l - location)
    (connected ?l1 ?l2 - location)
    (delivered ?p - package)
    (carrying ?r - robot ?p - package)
  )

  (:task deliver_all
    :parameters ()
  )
  (:task deliver_package
    :parameters (?p - package ?dest - location)
  )
  (:task go_to
    :parameters (?r - robot ?to - location)
  )

  (:method m_deliver_all_done
    :parameters ()
    :task (deliver_all)
    :precondition ()
    :ordered-subtasks ()
  )

  (:method m_deliver_all_step
    :parameters (?p - package ?dest - location)
    :task (deliver_all)
    :precondition (not (delivered ?p))
    :ordered-subtasks (and
      (t1 (deliver_package ?p ?dest))
      (t2 (deliver_all))
    )
  )

  (:method m_deliver_package
    :parameters (?r - robot ?p - package ?src - location ?dest - location)
    :task (deliver_package ?p ?dest)
    :precondition (and
      (robot_at ?r ?src)
      (package_at ?p ?src)
    )
    :ordered-subtasks (and
      (t1 (go_to ?r ?src))
      (t2 (pickup ?r ?p ?src))
      (t3 (go_to ?r ?dest))
      (t4 (drop ?r ?p ?dest))
    )
  )

  (:method m_go_already_there
    :parameters (?r - robot ?l - location)
    :task (go_to ?r ?l)
    :precondition (robot_at ?r ?l)
    :ordered-subtasks ()
  )

  (:method m_go_move
    :parameters (?r - robot ?from - location ?to - location)
    :task (go_to ?r ?to)
    :precondition (and
      (robot_at ?r ?from)
      (connected ?from ?to)
      (not (robot_at ?r ?to))
    )
    :ordered-subtasks (and
      (t1 (move ?r ?from ?to))
    )
  )

  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and
      (robot_at ?r ?from)
      (connected ?from ?to)
    )
    :effect (and
      (not (robot_at ?r ?from))
      (robot_at ?r ?to)
    )
  )

  (:action pickup
    :parameters (?r - robot ?p - package ?l - location)
    :precondition (and
      (robot_at ?r ?l)
      (package_at ?p ?l)
    )
    :effect (and
      (not (package_at ?p ?l))
      (carrying ?r ?p)
    )
  )

  (:action drop
    :parameters (?r - robot ?p - package ?l - location)
    :precondition (and
      (robot_at ?r ?l)
      (carrying ?r ?p)
    )
    :effect (and
      (not (carrying ?r ?p))
      (package_at ?p ?l)
      (delivered ?p)
    )
  )
)