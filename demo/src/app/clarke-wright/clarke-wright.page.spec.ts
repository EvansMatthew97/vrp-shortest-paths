import { CUSTOM_ELEMENTS_SCHEMA } from '@angular/core';
import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { ClarkeWrightPage } from './clarke-wright.page';

describe('ClarkeWrightPage', () => {
  let component: ClarkeWrightPage;
  let fixture: ComponentFixture<ClarkeWrightPage>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ ClarkeWrightPage ],
      schemas: [CUSTOM_ELEMENTS_SCHEMA],
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(ClarkeWrightPage);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
