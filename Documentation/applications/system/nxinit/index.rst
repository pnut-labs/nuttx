======================
``nxinit`` NXInit
======================

Overview
========

In NuttX, there are various options for the init entry point, with NSH being
one of the commonly used ones. However, taking NSH as an example, it is not
suitable for all usage scenarios. For instance, the code size required by
NSH and NSH_LIBRARY is relatively large, and NSH lacks management
capabilities for daemons/services (such as restarting services).

NXInit was created by Xiaomi to solve these issues. It is compatible with
most of the syntax of Android Init, which you can read about here:
https://android.googlesource.com/platform/system/core/+/master/init/README.md

It is lightweight, supports command execution triggered by events, supports
service/daemon management and many more.

The ``init.rc`` consists of five categories of statements, all line-oriented with
parameters separated by spaces. The content is processed by a preprocessor,
following C language specifications for escape rules and comment formats.

File path configured by ``CONFIG_SYSTEM_NXINIT_RC_FILE_PATH``.

1. Actions and Commands
-----------------------
.. code-block::

    on <trigger>
       <command>
       <command>
       <command>

- An Action (command sequence) contains trigger conditions. When satisfied,
  it is added to the execution queue.
- Commands are the specific execution content of actions, such as setprop
  (set system properties), start (start services), and mount (mount file
  systems).

2. Services and Options
-----------------------
.. code-block::

    service <name> <pathname> [ <argument> ]*
       <option>
       <option>
       ...

- A Service is a program started by Init, with a unique name (can be overridden
  via the override option).
- Options are modifiers for services, affecting their running mode and timing.
  Examples include class (specify service category for batch start/stop),
  override (override previously defined services), restart_period (interval
  for restarting exited services), and reboot_on_failure (critical services
  trigger device reboot on startup failure or abnormal exit).

Triggers
========

The triggers of an action serve as its triggering conditions, supporting
one or more triggers (which must all be satisfied simultaneously).

There are two types of triggers: event triggers (e.g., boot) and action
triggers (also known as property triggers, e.g., property:a=b). Actions can
include multiple action triggers but only one event trigger. Triggers can be
combined with && to represent "AND" conditions.

The value of a property trigger is matched with ``fnmatch``, so glob
patterns are accepted. Use ``!=`` to trigger when the value does not match,
e.g. ``property:a!=b``. An ``on <event>`` action fires only on the edge when
its condition becomes satisfied, not on every subsequent property update.

Property (action) trigger support requires the ``init_property_*()`` backend
to be provided; ``property_simple.c`` offers a minimal implementation that
forwards ``setprop`` to the action manager.

1. Event Trigger Execution Order
--------------------------------
1. boot: The first event after NXInit starts.
2. init: After boot event completation.
3. netinit: Optional, after netinit_bringup() returns.
4. finalinit: After all previous events complete.

2. Action Triggers Checking
---------------------------
- All action triggers are automatically checked once at startup.
- Property triggers are checked when the property is created or its value
  is updated (e.g., property:a=b is checked when a's value changes).

Built-in Properties
====================

NXInit sets one built-in property on its own, before the ``boot`` event
fires:

- ``sys.boot.reason``: the cause of the last board reset. It is queried
  once at startup via ``boardctl(BOARDIOC_RESET_CAUSE, ...)``; applications
  do not need to set it themselves. This requires
  ``CONFIG_BOARDCTL_RESET_CAUSE`` and a board-provided
  ``board_reset_cause()`` implementation.

  - If ``CONFIG_BOARDCTL_RESET_CAUSE`` is not enabled, ``sys.boot.reason``
    is never set, so any trigger referencing it (e.g.
    ``property:sys.boot.reason=...``) simply never matches; there is no
    dedicated "unset" value to trigger on.
  - If ``CONFIG_BOARDCTL_RESET_CAUSE`` is enabled but the ``boardctl()``
    call itself fails (the board reports an error), NXInit aborts startup.

  The value has one of two forms:

  1. ``<cause>,<subreason>`` for hardware reset causes, where
     ``<subreason>`` is a numeric flag (e.g. the watchdog timer index):
     ``cold``, ``watchdog``, ``undervoltage``, ``warm``, ``powerkey``,
     ``lowpower``, ``unknown``. For example, a watchdog reset from timer 4
     sets the property to ``watchdog,4``.
  2. A single soft-reset reason string, for software-triggered resets
     (``BOARDIOC_RESETCAUSE_CPU_SOFT``): ``reboot``, ``assert``,
     ``kernel_panic``, ``bootloader``, ``recovery``, ``factory_reset``,
     ``factory_reset_inquiry``, ``thermal``.

  Since the property is matched with ``fnmatch`` and alternatives can be
  separated with ``|``, a trigger can match on the cause, the subreason, or
  a set of soft-reset reasons:

  .. code-block::

      on init && property:sys.boot.reason=watchdog,4
          ...

      on init && property:sys.boot.reason=bootloader|recovery|thermal
          ...

Service States
==============

A service is in one of five states: ``starting``, ``ready``, ``stopping``,
``restarting`` or ``stopped``. A service is ``ready`` as soon as it runs,
unless it is declared with the option ``notify``: then it is ``starting``
until its own task says it is ready on the control socket (below), and
``starting`` again after each restart. ``notify`` needs the control socket
(``CONFIG_SYSTEM_NXINIT_CONTROL``); without it the option is unknown, an
``init.rc`` using it fails to parse, and NXInit stops at the error.

Every change of state sets the property ``svc.<name>.state``, so that a
service can be started once another is ready:

.. code-block::

    service settings settingsd
        notify

    service modem modemd

    on init
        start settings

    on property:svc.settings.state=ready
        start modem

A property trigger fires each time its condition becomes true, so ``modem``
is started again, if it is not running, after ``settings`` restarts and
says it is ready again.

A service declared with ``gentle_kill`` is sent ``SIGTERM`` when it is
stopped, and ``SIGKILL`` if it has not exited
``CONFIG_SYSTEM_NXINIT_SERVICE_GENTLE_KILL_TIMEOUT`` milliseconds later
(200 by default): a service that saves its state as it stops needs as long
as that takes.

The Control Socket
==================

With ``CONFIG_SYSTEM_NXINIT_CONTROL``, NXInit listens on a local stream
socket, ``CONFIG_SYSTEM_NXINIT_CONTROL_PATH`` (``/var/run/nxinit`` by
default), through which programs ask for the services' states, follow
their changes, find which service a task is, start and stop services, and
say that they are ready. It speaks lines of text: a connection first
receives a version line, ``nxinit 1``; then each line it sends is a
command, answered ``ok ...`` or ``error <errno> <text>``.

=====================  ====================================================
Command                Answer
=====================  ====================================================
``state``              ``ok <n>``, then n lines ``<service> <state> <pid>``
``state <service>``    ``ok <service> <state> <pid>``
``watch``              ``ok``; then, as long as the connection is open, a
                       line ``event <service> <state> <pid>`` for every
                       change of state
``who <pid>``          ``ok <service>``: the service whose task that is
``start <service>``    ``ok``, also when it runs already
``stop <service>``     ``ok``
``ready``              ``ok``: the caller's service is ready
=====================  ====================================================

The pid is 0 when the service is not running. On a connection that
watches, an event line may come at any time, also between a command and its
answer; the n lines after ``ok <n>`` come together. Lines are shorter than
80 characters; a longer command is answered ``error 7 line too long`` and
the connection closed. NXInit never waits on a client: one that does not
read what it is sent is closed, and one more than
``CONFIG_SYSTEM_NXINIT_CONTROL_CLIENTS`` gets ``error 16 too many
clients`` in place of the version line, and is closed. When the socket
cannot be set up, NXInit logs it and goes on without it.

``ready`` is taken only from the task NXInit started for a ``notify``
service, not from a task that one starts: NXInit learns the caller with
``SO_PEERCRED`` (the option selects ``CONFIG_NET_LOCAL_SCM``). A caller it
cannot name gets ``error 1 caller unknown``, a task that is no running
service's ``error 3 not a running service``, and a service without
``notify`` ``error 22 not a notify service``.

The ``svc`` command (``CONFIG_SYSTEM_NXINIT_SVC``) speaks the protocol from
the shell:

.. code-block::

    nsh> svc state
    ok 2
    console ready 7
    settings ready 8
    nsh> svc who 7
    ok console
    nsh> svc watch
    ok
    event settings stopping 8
    event settings stopped 0
    ...

``svc watch`` prints the changes until NXInit closes the connection; above,
``svc stop settings`` was run from another shell meanwhile.

``svc ready`` says a service is ready when ``svc`` itself is the service's
task (``service x svc ready``), not when a shell or program the service
runs starts it.

Commands
========
The commands supported by an action fall into three types: the built-in
commands of NXInit, the built-in commands of NSH (if NSH is enabled), and
Builtin Apps.

The following is an explanation of some of NXInit's built-in commands.

- System Operations: mount/umount (mount/unmount file systems), setprop
  (set system properties), chmod/chown (modify file permissions/owner).
- Service Management: start/stop/restart (start/stop/restart services),
  class_start/class_stop (batch start/stop service categories).
- File Operations: mkdir (create directories with configurable permissions,
  owner, and encryption settings), copy/write (copy files/write file content).
- Others: trigger (trigger events).

Compound Commands
------------------

A single line inside an action body may chain multiple commands with
``&&`` and ``||``, using the familiar shell short-circuit semantics: the
next command in an ``&&`` chain only runs if the previous one exited with
status 0 (success), and the next command in an ``||`` chain only runs if
the previous one exited non-zero (failure). A quoted argument
(``"..."``) may contain ``&&``/``||`` without being treated as an
operator.

.. code-block::

    on boot
        echo "start" && hello && echo "done"
        ls /missing || echo "not found"
        echo "A" && echo "B" || echo "fallback"

Examples
========
This is an example of enabling the basic functions of the NXInit component,
with all log levels additionally enabled to facilitate debugging of the
init.rc script.

defconfig:

.. code-block:: diff

    +CONFIG_INIT_ENTRYPOINT="init_main"
    +CONFIG_SYSTEM_NXINIT=y
    +CONFIG_SYSTEM_NXINIT_DEBUG=y
    +CONFIG_SYSTEM_NXINIT_INFO=y
    +CONFIG_SYSTEM_NXINIT_WARN=y

init.rc:

.. code-block::

    on boot
        start console        /* Start the service named "console" */
        sleep 1              /* Block for 1 second */
        exec_start mkdir_tmp /* Start the "mkdir_tmp" service and wait for exit */

    on boot && property:sys.boot.reason=bootloader
        echo "On boot, the reason is BL."
        start fastboot

    service console sh
        class core
        restart_period 1000  /* Service restart interval (in milliseconds), calculated from service start time */

    service fastboot fastbootd
        class core
        restart_period 7000

    service mkdir_tmp sh -c "mkdir /tmp"
        reboot_on_failure 0 /* Reboot device with reason 0 on startup failure or abnormal exit */
        oneshot             /* This service runs only once; releases resources after exit */

