#Use Cases

## Use Case: 5 Buy Goods

###CHARACTERISTIC INFORMATION

**Goal in Context:** Buyer issues request directly to our company, expects goods shipped and to be billed.

**Scope:** Company

**Level:** Summary

**Preconditions:** We know Buyer, their address, etc.

**Success End Condition:** Buyer has goods, we have money for the goods.

**Failed End Condition:** We have not sent the goods, Buyer has not spent the money.

**Primary Actor:** Buyer, any agent (or computer) acting for the customer

**Trigger:** purchase request comes in.

###MAIN SUCCESS SCENARIO

1. Buyer calls in with a purchase request.

2. Company captures buyer’s name, address, requested goods, etc.

3. Company gives buyer information on goods, prices, delivery dates, etc.

4. Buyer signs for order.

5. Company creates order, ships order to buyer.

6. Company ships invoice to buyer.

7. Buyers pays invoice.

###EXTENSIONS

* 3a. Company is out of one of the ordered items:

  + 3a1. Renegotiate order.

* 4a. Buyer pays directly with credit card:

  + 4a1. Take payment by credit card (use case 44)

* 7a. Buyer returns goods:

  + 7a1. Handle returned goods (use case 105)

###SUB-VARIATIONS

* 1a. Buyer may use
phone in,
fax in,
use web order form,
electronic interchange

* 7a. Buyer may pay by
cash or money order
check
credit card

###RELATED INFORMATION

**Priority:** top

**Performance Target:** 5 minutes for order, 45 days until paid

**Frequency:** 200/day

**Superordinate Use Case:** Manage customer relationship (use case 2)

**Subordinate Use Cases:**

* Create order (use case 15)

* Take payment by credit card (use case 44)

* Handle returned goods (use case 105)

**Channel to primary actor:** may be phone, file or interactice

**Secondary Actors:** credit card company, bank, shipping service

**Channels to Secondary Actors:**

###OPEN ISSUES

* What happens if we have part of the order?

* What happens if credit card is stolen?

###SCHEDULE

Due Date: release 1.0