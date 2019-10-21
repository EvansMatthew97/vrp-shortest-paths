import { CUSTOM_ELEMENTS_SCHEMA } from '@angular/core';
import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { PredictiveRoutesPage } from './predictive-routes.page';

describe('PredictiveRoutesPage', () => {
  let component: PredictiveRoutesPage;
  let fixture: ComponentFixture<PredictiveRoutesPage>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ PredictiveRoutesPage ],
      schemas: [CUSTOM_ELEMENTS_SCHEMA],
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(PredictiveRoutesPage);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
